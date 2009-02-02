#ifndef RTRACER_FRAME_COUNTER_H
#define RTRACER_FRAME_COUNTER_H

class FrameCounter
{
public:
	FrameCounter();
	void NextFrame();
	inline double FPS() const { return fps; }

private:
	unsigned frames;
	double time,fps;
};

#endif

