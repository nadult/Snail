#include "image.h"


Image::Image() :width(0),height(0) {
}

Image::Image(size_t w,size_t h,size_t align) {
	width =((w+align-1)/align)*align;
	height=((h+align-1)/align)*align;
	buffer.resize(width*height*3);
}

void Image::SaveToFile(const char *fileName) {
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

void Image::LoadFromFile(const char *fileName) {
	FILE *fptr=fopen(fileName,"rb");
	if(!fptr) throw Exception("Error while opening image file.");

	getc(fptr); getc(fptr); getc(fptr);
	getc(fptr); getc(fptr);
	getc(fptr); getc(fptr);
	getc(fptr);
	getc(fptr); getc(fptr);
	getc(fptr); getc(fptr);

	unsigned short w,h;
	fread(&w,2,1,fptr);
	fread(&h,2,1,fptr);
	getc(fptr); getc(fptr);
	width=w; height=h;
	buffer.resize(width*height*3);

	for(int n=height-1;n>=0;n--)
		fread(&buffer[0]+n*width*3,width*3,1,fptr);

	fclose(fptr);
}

void Image::Pixel(int x,int y,char r,char g,char b) {
	x=Clamp(x,0,int(width)-1); y=Clamp(y,0,int(height)-1);
	char *p=&buffer[(x+y*width)*3];
	p[0]=r; p[1]=g; p[2]=b;
}
