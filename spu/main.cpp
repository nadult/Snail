#include <mutex_init.h>
#include <mutex_lock.h>
#include <mutex_unlock.h>

#include <cond_init.h>
#include <cond_signal.h>
#include <cond_wait.h>
#include <cond_broadcast.h>

#include "spu/base.h"
#include "spu/stats.h"

#define PROGRESSIVE

extern TaskInfo info;
extern Stats stats;

int ProcessTask();

TasksInfo tasks ALIGN256;

int main(unsigned long long, unsigned long long ptasks, unsigned long long speid) {
	Mem2Local(ptasks, &tasks, sizeof(TasksInfo));
	bool taskFinished = 0;

	while(true) {
		bool process = 0, wake = 0;
		spu_write_decrementer(0xfffffffe);

		_mutex_lock(tasks.mutex);
		Mem2Local(ptasks, &tasks, sizeof(TasksInfo));
		if(!taskFinished && tasks.pleaseDie[speid]) {
			_mutex_unlock(tasks.mutex);
			return 0;
		}

		if(taskFinished) {
			tasks.nFinished++;
			if(tasks.nTasks == tasks.nAllTasks)
				tasks.offsets[speid] = tasks.nAllTasks;
#ifndef PROGRESSIVE
			if(tasks.nFinished == tasks.nAllTasks)
				wake = 1;
#endif

			tasks.nTrisIntersected += stats.intersects;
			tasks.nBVHIterations += stats.iters;
			tasks.nRaysTraced += stats.rays;
			for(int k = 0, count = Min(sizeof(gTimers), sizeof(tasks.timers)) / 4; k < count; k++) {
				tasks.timers[k] += gTimers[k];
				gTimers[k] = 0;
			}
			stats = Stats();

			taskFinished = 0;
		}
//		else if(!tasks.nTasks)
//			_cond_wait(tasks.newTaskCond, tasks.mutex);

		if(tasks.nTaken < tasks.nTasks) {
			Mem2Local(tasks.taskList + tasks.nTaken++ * sizeof(TaskInfo), &info, sizeof(TaskInfo));
#ifdef PROGRESSIVE
			tasks.offsets[speid] = info.dataOffset;
#endif
			process = 1;
		}

		Local2Mem(ptasks, &tasks, sizeof(TasksInfo));
#ifndef PROGRESSIVE
		if(wake)
			_cond_signal(tasks.finishCond);
#endif
		_mutex_unlock(tasks.mutex);
		if(process) {
			ProcessTask();
			taskFinished = 1;
		}
	}

	return 0;
}
