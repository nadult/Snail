#ifndef THREAD_POOL_H
#define THREAD_POOL_H

#include <vector>

#if defined(__PPC) || defined(__PPC__)
#include "spu/context.h"
#endif


namespace thread_pool {

#if defined(__PPC) || defined(__PPC__)
	struct Task {
		Task() :preload(0) { }
		virtual ~Task() { }
		virtual void Work(SPEContext *context) = 0;

		spe_program_handle_t *preload;
	};

#else
	struct Task {
		virtual ~Task() { }
		virtual void Work() = 0;
	};
#endif

	void Run(void *tasks, int nTasks, int stride, int nThreads);

	template <class TTask>
	void Run(std::vector<TTask> &ttasks, int nThreads) {
		Run(&ttasks[0], ttasks.size(), sizeof(TTask), nThreads);
	}

}

#endif
