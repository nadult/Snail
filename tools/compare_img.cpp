#include <stdio.h>
#include "rtracer.h"

int main(int argc,char **argv)
{
	if(argc<3) {
		printf("Usage\ncompare_img image1 image2\n");
		return 0;
	}
	Image img[2];
	img[0].LoadFromFile(argv[1]);
	img[1].LoadFromFile(argv[2]);

	long long sumr[2]={0,},sumg[2]={0,},sumb[2]={0,};
	for(int n=0;n<2;n++) {
		int w=img[n].width,h=img[n].height;
		for(int y=0;y<h;y++) {
			unsigned char *line=(unsigned char*)&img[n].buffer[y*w*3];
			for(int x=0;x<w;x++) {
				sumr[n]+=line[x*3+0];
				sumg[n]+=line[x*3+1];
				sumb[n]+=line[x*3+2];
			}
		}

		double icount=1.0/(double(w)*double(h));
		printf("%s:\t\tR:%.2f\tG:%.2f\tB:%.2f\n",argv[n+1],
				double(sumr[n])*icount,double(sumg[n])*icount,double(sumb[n])*icount);
	}


	return 0;
}
