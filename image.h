#ifndef RTRACER_IMAGE_H
#define RTRACER_IMAGE_H

#include "rtbase.h"

class Image
{
public:
	Image();
	Image(size_t w,size_t h);
	void SaveToFile(const char*fileName);
	void LoadFromFile(const char *fileName);
	void Pixel(int x,int y,char r,char g,char b);

	vector<char> buffer;
	size_t width,height;
};

#endif

