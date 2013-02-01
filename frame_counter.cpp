#include "frame_counter.h"
#include "baselib.h"

FrameCounter::FrameCounter()
	:time(baselib::getTime()), frames(0), fps(0) {
}

void FrameCounter::NextFrame() {
	double tTime = baselib::getTime();
	double interval = 0.5f;

	frames++;
	if(tTime - time > interval) {
		fps = double(frames) / (tTime - time);
		time = tTime;
		frames = 0;
	}
}

