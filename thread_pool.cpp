#include <pthread.h>
#include <iostream>
#include <baselib.h>
#include "thread_pool.h"

enum { maxThreads = 32 };

#if defined(__PPC) || defined(__PPC__)
#else
#define USE_SPINLOCKS
#endif


namespace thread_pool {

namespace {

#if defined(__PPC) || defined(__PPC__)
	SPEContext speContext[maxThreads];
	SPEGangContext speGang;
	spe_program_handle_t *speProgram[maxThreads];
#endif
	pthread_t threads[maxThreads];
	pthread_mutex_t mutexes[maxThreads];
	pthread_cond_t sleeping[maxThreads];

#ifdef USE_SPINLOCKS
	pthread_spinlock_t spinlock;
#else
	pthread_mutex_t lock;
#endif
	volatile bool diePlease[maxThreads];
	volatile int nextTask = 0;
	volatile int nFinished = 0;
	std::vector<Task*> tasks;
	int nThreads = 0;

	Task *GetTask(int id, bool notFirst) __attribute__((noinline));
	Task *GetTask(int id, bool notFirst) {
		if(notFirst) {
			if(nFinished == tasks.size() - 1) {
				pthread_mutex_lock(&mutexes[0]);
				nFinished++;
				pthread_cond_signal(&sleeping[0]);
				pthread_mutex_unlock(&mutexes[0]);
			}
			else nFinished++;
		}
		return nextTask == tasks.size()? 0 : tasks[nextTask++];
	}

	void *InnerLoop(void *id_) {	
		int id = (long long)id_;
		Task *task = 0;
	REPEAT:
		while(true) {
#ifdef USE_SPINLOCKS
			pthread_spin_lock(&spinlock);
			task = GetTask(id, task);
			pthread_spin_unlock(&spinlock);
#else
			pthread_mutex_lock(&lock);
			task = GetTask(id, task);
			pthread_mutex_unlock(&lock);
#endif
			if(!task) break;

#if defined(__PPC) || defined(__PPC__)
			if(task->preload && speProgram[id] != task->preload) {
				speProgram[id] = task->preload;
				speContext[id].Load(task->preload);
			}
			speProgram[id] = task->preload;
			task->Work(&speContext[id]);
#else
			task->Work();
#endif
		}

		if(id && !diePlease[id]) {
			pthread_mutex_lock(&mutexes[id]);
			pthread_cond_wait(&sleeping[id], &mutexes[id]);
			pthread_mutex_unlock(&mutexes[id]);
			goto REPEAT;
		}

		return 0;
	}

	void FreeThreads();

	void ChangeThreads(int nThreads_) {
		Assert(nThreads_ > 0 && nThreads_ <= maxThreads);

		static bool sinit = 0;
		if(!sinit) {
			tasks.reserve(1024);
#if defined(__PPC) || defined(__PPC__)
			speGang.Create();
#endif

#ifdef USE_SPINLOCKS
			pthread_spin_init(&spinlock, PTHREAD_PROCESS_PRIVATE);
#else
			pthread_mutex_init(&lock, 0);
#endif
			atexit(FreeThreads);
			sinit = 1;
		}

		for(; nThreads < nThreads_; nThreads++) {
			diePlease[nThreads] = 0;
			pthread_mutex_init(&mutexes[nThreads], 0);
			pthread_cond_init(&sleeping[nThreads], 0);
#if defined(__PPC) || defined(__PPC__)
			speContext[nThreads].Create(&speGang);
			speProgram[nThreads] = 0;
#endif
			if(nThreads) pthread_create(&threads[nThreads], 0, InnerLoop, (void*)nThreads);
		}
		while(nThreads > nThreads_) {
			nThreads--;
			if(nThreads) {
				diePlease[nThreads] = 1;
				pthread_cond_signal(&sleeping[nThreads]);
				pthread_join(threads[nThreads], 0);
			}
			pthread_mutex_destroy(&mutexes[nThreads]);
			pthread_cond_destroy(&sleeping[nThreads]);
#if defined(__PPC) || defined(__PPC__)
			speContext[nThreads].Destroy();
#endif
		}
	}

	void FreeThreads() {
		ChangeThreads(0);
#ifdef USE_SPINLOCKS
		pthread_spin_destroy(&spinlock);
#else
		pthread_mutex_destroy(&lock);
#endif
#if defined(__PPC) || defined(__PPC__)
		speGang.Destroy();
#endif
	}

}

	void Run(void *ptasks, int nTasks, int stride, int nThreads_) {
		ChangeThreads(nThreads_);

#ifdef USE_SPINLOCKS
		pthread_spin_lock(&spinlock);
#else
		pthread_mutex_lock(&lock);
#endif
			tasks.resize(nTasks);
			for(int n = 0; n < nTasks; n++)
				tasks[n] = (Task*)(((char*)ptasks) + stride * n);
			nFinished = 0;
			nextTask = 0;
#ifdef USE_SPINLOCKS
		pthread_spin_unlock(&spinlock);
#else
		pthread_mutex_unlock(&lock);
#endif

		for(int n = 1; n < nThreads; n++)
			pthread_cond_broadcast(&sleeping[n]);
		InnerLoop(0);
	
		if(nFinished != tasks.size()) {
			pthread_mutex_lock(&mutexes[0]);
			if(nFinished != tasks.size())
				pthread_cond_wait(&sleeping[0], &mutexes[0]);
			pthread_mutex_unlock(&mutexes[0]);
		}
	}

}

