#ifndef THREAD_POOL_H
#define THREAD_POOL_H

#include <vector>

namespace thread_pool {

	struct Task {
		virtual ~Task() { }
		virtual void Work() = 0;
	};

	void Run(void *tasks, int nTasks, int stride, int nThreads);

	template <class TTask>
	void Run(std::vector<TTask> &ttasks, int nThreads) {
		Run(&ttasks[0], ttasks.size(), sizeof(TTask), nThreads);
	}

}

#endif
