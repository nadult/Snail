#ifndef RTRACER_TASK_SWITCHER_H
#define RTRACER_TASK_SWITCHER_H

#include <baselib.h>

int GetCoresNum();

template <class Task>
class TaskSwitcher
{
public:
	TaskSwitcher(int tasksReserve=0);
	void AddTask(const Task&);
	void Work(int nThreads=0) NOINLINE;

private:
	vector<Task> tasks;
};

template <class Task>
TaskSwitcher<Task>::TaskSwitcher(int tasksReserve) {
	tasks.reserve(tasksReserve);
}

template <class Task>
void TaskSwitcher<Task>::AddTask(const Task &task) {
	tasks.push_back(task);
}

template <class Task>
void TaskSwitcher<Task>::Work(int numThreads) {
	if(numThreads==0)
		numThreads=GetCoresNum();

	int nextTask=0;

#pragma omp parallel for
	for(int n=0;n<numThreads;n++) {
		while(true) {
			Task *task;
			bool finished=0;
#pragma omp critical
			{
				if(nextTask<tasks.size()) task=&tasks[nextTask++];
				else finished=1;
			}

			if(finished) break;
			task->Work();
		}
	}
}

#endif

