#include "frame_counter.h"
#include <fwk/sys_base.h>

FrameCounter::FrameCounter() : frames(0), time(fwk::getTime()), fps(0) {}

void FrameCounter::NextFrame() {
	double tTime = fwk::getTime();
	double interval = 0.5f;

	frames++;
	if(tTime - time > interval) {
		fps = double(frames) / (tTime - time);
		time = tTime;
		frames = 0;
	}
}
