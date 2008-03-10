#ifndef SDL_OUTPUT_H
#define SDL_OUTPUT_H

#include "rtbase.h"
#include "image.h"

class SDLOutput
{
public:
	SDLOutput(int w,int h,bool full);
	virtual ~SDLOutput();

	virtual void Render(const Image &img);
	virtual bool PollEvents();
	virtual bool TestKey(u8 c);
	virtual int MouseDX();
	virtual int MouseDY();
};

#endif

