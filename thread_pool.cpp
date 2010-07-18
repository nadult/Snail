#include <pthread.h>
#include <iostream>
#include <baselib.h>
#include "thread_pool.h"

//TODO tu gdzies jest bug, bo podczas renderowania na > 1 watku czasami
//sie zwiesza
enum { maxThreads = 16 };

namespace thread_pool {

namespace {

	pthread_t threads[maxThreads];
	pthread_mutex_t mutexes[maxThreads];
	pthread_cond_t sleeping[maxThreads];
	pthread_spinlock_t spinlock;
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
			pthread_spin_lock(&spinlock);
			task = GetTask(id, task);
			pthread_spin_unlock(&spinlock);
			if(!task) break;

			task->Work();
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
			pthread_spin_init(&spinlock, PTHREAD_PROCESS_PRIVATE);
			atexit(FreeThreads);
			sinit = 1;
		}

		for(; nThreads < nThreads_; nThreads++) {
			diePlease[nThreads] = 0;
			pthread_mutex_init(&mutexes[nThreads], 0);
			pthread_cond_init(&sleeping[nThreads], 0);
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
		}
	}

	void FreeThreads() {
		ChangeThreads(0);
		pthread_spin_destroy(&spinlock);
	}

}

	void Run(void *ptasks, int nTasks, int stride, int nThreads_) {
		ChangeThreads(nThreads_);

		pthread_spin_lock(&spinlock);
			tasks.resize(nTasks);
			for(int n = 0; n < nTasks; n++)
				tasks[n] = (Task*)(((char*)ptasks) + stride * n);
			nFinished = 0;
			nextTask = 0;
		pthread_spin_unlock(&spinlock);

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

