#include "stdafx.h"
#include "rtracer.h"
#include <SDL/SDL.h>
#include <string>

#pragma comment(lib,"SDL.lib")

namespace
{
	bool initiated=0;
	SDL_Surface *screen;
	char lastKey=0;
	char keybuf[256];
	int mdx,mdy,resx,resy;
}

SDLOutput::SDLOutput(int w, int h,bool full)
{
	resx=w; resy=h;
	if(initiated) throw Exception("SDLOutput alredy created");
  
    if(SDL_Init(SDL_INIT_VIDEO)<0) throw Exception("Error while initializing SDL");
   
	if (!(screen = SDL_SetVideoMode(w, h,32,(full?SDL_FULLSCREEN:0)|SDL_SWSURFACE))) {
        SDL_Quit();
		throw Exception("Error while setting video mode");
    }
	memset(keybuf,0,256);
//	SDL_ShowCursor(0);
//	SDL_WM_GrabInput(SDL_GRAB_ON);
	initiated=1;
}

SDLOutput::~SDLOutput()
{
	initiated=0;
//	SDL_ShowCursor(1);
	SDL_Quit();
}

void SDLOutput::Render(const Image &img)
{
    if(SDL_MUSTLOCK(screen))
        if(SDL_LockSurface(screen)<0)
			throw Exception("Error while locking surface");

	int w=Min(screen->w,img.width),h=Min(screen->h,img.height);
	for(int y=0;y<h;y++) {
			i8 *dst=(char*)screen->pixels+screen->pitch*y;
			i8 *src=(char*)&img.buffer[y*img.width*3];
		for(int x=0;x<w;x++) { i8 r=*src++,g=*src++,b=*src++; *dst++=b; *dst++=g; *dst++=r; dst++; }
	}

    if(SDL_MUSTLOCK(screen)) SDL_UnlockSurface(screen);
    SDL_Flip(screen);
}

bool SDLOutput::PollEvents()
{
	SDL_Event event;
	mdx=0; mdy=0;
	
	while(SDL_PollEvent(&event)) {      
		switch (event.type) {
		case SDL_QUIT: return 0;
		case SDL_KEYDOWN: keybuf[event.key.keysym.sym]=1; break;
		case SDL_KEYUP: keybuf[event.key.keysym.sym]=0; break;
		case SDL_MOUSEMOTION: mdx=event.motion.xrel; mdy=event.motion.yrel; break;
		}
	}

	return 1;
}

bool SDLOutput::TestKey(u8 c)
{
	return keybuf[c]==1;
}

int SDLOutput::MouseDX()
{
	return mdx;
}

int SDLOutput::MouseDY()
{
	return mdy;
}
