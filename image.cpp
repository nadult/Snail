#include "stdafx.h"
#include "rtracer.h"


Image::Image(size_t w,size_t h)
	:width(w),height(h),buffer(w*h*3)
{
}

void Image::SaveToFile(const char *fileName)
{
	FILE *fptr=fopen(fileName,"wb");

	putc(0,fptr); putc(0,fptr); putc(2,fptr);
	putc(0,fptr); putc(0,fptr);
	putc(0,fptr); putc(0,fptr);
	putc(0,fptr);
	putc(0,fptr); putc(0,fptr);
	putc(0,fptr); putc(0,fptr);
	putc((width & 0x00FF),fptr);
	putc((width & 0xFF00) / 256,fptr);
	putc((height & 0x00FF),fptr);
	putc((height & 0xFF00) / 256,fptr);
	putc(24,fptr);
	putc(0,fptr);

	for(int n=height-1;n>=0;n--)
		fwrite(&buffer[0]+n*width*3,width*3,1,fptr);

	fclose(fptr);
}

void Image::Pixel(int x,int y,char r,char g,char b)
{
	x=Clamp(x,0,int(width)-1); y=Clamp(y,0,int(height)-1);
	char *p=&buffer[(x+y*width)*3];
	p[0]=r; p[1]=g; p[2]=b;
}
