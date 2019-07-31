#pragma once

class FrameCounter
{
public:
	FrameCounter();
	void NextFrame();
	inline double FPS() const { return fps; }

private:
	unsigned frames;
	double time, fps;
};
