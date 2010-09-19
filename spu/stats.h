#ifndef RTRACER_SPU_STATS_H
#define RTRACER_SPU_STATS_H

#include "spu/base.h"

extern int gTimers[16];

enum {
	timerRendering = 0,
	timerTracing = 1,
	timerIntersecting = 2,
	timerShading = 3,
	timerSampling = 4,
};

struct BlockTimer {
	BlockTimer(int timerId) :timerId(timerId), timestamp(spu_read_decrementer()) { }
	void End() {
		gTimers[timerId] += timestamp - spu_read_decrementer();
		timestamp = 0xffffffff;
	}
	void Begin() {
		timestamp = spu_read_decrementer();
	}
	~BlockTimer() {
		if(timestamp != 0xffffffff)
			gTimers[timerId] += timestamp - spu_read_decrementer();
	}

	const int timerId;
	unsigned timestamp;
};

struct Stats {
	Stats() :iters(0), intersects(0), rays(0) { }
	Stats(int it, int in, int r) :iters(it), intersects(in), rays(r) { }

	const Stats operator-(const Stats &rhs) const {
		return Stats(iters - rhs.iters, intersects - rhs.intersects, rays - rhs.rays);
	}
	
	const Stats operator+(const Stats &rhs) const {
		return Stats(iters + rhs.iters, intersects + rhs.intersects, rays + rhs.rays);
	}

	int iters, intersects;
	int rays;
};

#endif
