#include "frame_counter.h"
#include "baselib.h"

FrameCounter::FrameCounter()
	:time(baselib::GetTime()),fps(0),frames(0) {
}

void FrameCounter::NextFrame() {
	double tTime=baselib::GetTime();
	double interval=0.5f;

	frames++;
	if(tTime-time>interval) {
		fps=double(frames)/interval;
		time+=interval;
		frames=0;
	}
}

