#include <map>
#include <iostream>
#include <memory.h>

#include "ray_generator.h"
#include "task_switcher.h"
#include "scene.h"
#include "camera.h"

#include "bihtree.h"
#include "kdtree.h"

#include "gl_window.h"

using std::cout;
using std::endl;

struct Options {
	Options(bool pixD,bool gd,bool refl,bool rdtsc) :pixDoubling(pixD),grid(gd),reflections(refl),rdtscShader(rdtsc) { }
	Options() { memset(this,0,sizeof(Options)); }

	bool pixDoubling,grid;
	bool reflections,rdtscShader;
};


template <class Scene,int QuadLevels>
struct GenImageTask {
	GenImageTask(const Scene *scn,const Camera &cam,Image *tOut,const Options &opt,uint tx,uint ty,uint tw,uint th,TreeStats *outSt)
		:scene(scn),camera(cam),out(tOut),options(opt),startX(tx),startY(ty),width(tw),height(th),outStats(outSt) {
		}

	uint startX,startY;
	uint width,height;

	TreeStats *outStats;
	const Scene *scene;
	Camera camera;
	Image *out;
	Options options;

	void Work() {
		float ratio=float(out->width)/float(out->height);

		enum { NQuads=1<<(QuadLevels*2), PWidth=2<<QuadLevels, PHeight=2<<QuadLevels };

		Matrix<Vec4f> rotMat(Vec4f(camera.right),Vec4f(camera.up),Vec4f(camera.front),Vec4f(0,0,0,1));
		rotMat=Transpose(rotMat);

		Vec3q origin; Broadcast(camera.pos,origin);
		RayGenerator rayGen(QuadLevels,out->width,out->height,camera.plane_dist);

		uint pitch=out->width*3;
		u8 *outPtr=(u8*)&out->buffer[startY*pitch+startX*3];

		for(int y=0;y<height;y+=PHeight) {
			for(int x=0;x<width;x+=PWidth) {
				Vec3q dir[NQuads],idir[NQuads];
				rayGen.Generate(PWidth,PHeight,startX+x,startY+y,dir);

				for(int n=0;n<NQuads;n++) {
					Vec3f tmp[4]; Convert(dir[n],tmp);
					for(int k=0;k<4;k++) tmp[k]=rotMat*tmp[k];
					Convert(tmp,dir[n]);
					dir[n]*=RSqrt(dir[n]|dir[n]);
					idir[n]=VInv(dir[n]);
				}

				TracingContext<Scene,RayGroup<NQuads,1,1>,RaySelector<NQuads> > context(*scene,RayGroup<NQuads,1,1>(dir,&origin,idir));
				Vec3q *rgb=context.color;

				context.options=TracingOptions(options.reflections?1:0,options.rdtscShader);
				
				scene->RayTracePrimary(context);
				outStats->Update(context.stats);

				rayGen.Decompose(rgb,rgb);
				Vec3f trgb[PWidth*PHeight];
				for(int q=0;q<NQuads;q++)
					Convert(VClamp(	rgb[q]*Const<floatq,255>(),
									Vec3q(Const<floatq,0>()),
									Vec3q(Const<floatq,255>())),trgb+q*4);

				int tWidth =Min(x+PWidth +startX,out->width )-startX-x;
				int tHeight=Min(y+PHeight+startY,out->height)-startY-y;
				for(int ty=0;ty<tHeight;ty++) {
					u8 *ptr=outPtr+x*3+(ty+y)*pitch;

					for(int tx=0;tx<tWidth;tx++) {
						ptr[0]=trgb[tx+ty*PWidth].x;
						ptr[1]=trgb[tx+ty*PWidth].y;
						ptr[2]=trgb[tx+ty*PWidth].z;
						ptr+=3;
					}
				}
			}
		}

		if(options.grid) for(int y=0;y<height;y++) {
			int addX=y%PHeight==0?1:PWidth;
			unsigned char *buf=(unsigned char*)&out->buffer[((startY+y)*out->width+startX)*3];

			for(int x=0;x<width;x+=addX)
				{ buf[x*3+0]=255; buf[x*3+1]=0; buf[x*3+2]=0; }
		}
	}
};


template <int QuadLevels,class Scene>
TreeStats GenImage(const Scene &scene,const Camera &camera,Image &image,const Options options,uint tasks) {
	enum { taskSize=64 };

	uint numTasks=(image.width+taskSize-1)*(image.height+taskSize-1)/(taskSize*taskSize);

	vector<TreeStats> taskStats(numTasks);

	TaskSwitcher<GenImageTask<Scene,QuadLevels> > switcher(numTasks);
	uint num=0;
	for(uint y=0;y<image.height;y+=taskSize) for(uint x=0;x<image.width;x+=taskSize) {
		switcher.AddTask(GenImageTask<Scene,QuadLevels> (&scene,camera,&image,options,x,y,
					Min(taskSize,image.width-x),Min(taskSize,image.height-y),
					&taskStats[num]) );
		num++;
	}

	switcher.Work(tasks);

	TreeStats stats;
	for(uint n=0;n<numTasks;n++)
		stats.Update(taskStats[n]);
	return stats;
}

void PrintBaseSizes() {
#define PRINT_SIZE(cls)		{printf("sizeof(" #cls "): %d\n",sizeof(cls));}
	PRINT_SIZE(KDNode)
	PRINT_SIZE(BIHNode)
	PRINT_SIZE(Triangle)
	PRINT_SIZE(Object)
	PRINT_SIZE(Sphere)
#undef PRINT_SIZE
}

Camera GetDefaultCamera(string model) {
	static bool initialized=0;
	static std::map< string, Camera> cams;
	if(!initialized) {
		initialized=1;
			// pompei?
		Camera pompei(
			Vec3f(52.423584,158.399719,51.276756),
			Vec3f(0.999916,0.000000,-0.013203),
			Vec3f(-0.013203,0.000000,-0.999916));

		// abrams, lancia from the side
		Camera abrams(
			Vec3f(-125.014099,-7.600281,115.258301),
			Vec3f(0.907629,0.000000,-0.419782),
			Vec3f(-0.419782,0.000000,-0.907629));

		Camera abramsTop( Vec3f(-118.3508,-93.6003,86.6569), Vec3f(0.8508,0.0000,-0.5254), Vec3f(-0.5254,0.0000,-0.8508) );
		Camera abramsLeak( Vec3f(102.7247,-31.6003,-76.6981), Vec3f(-0.8534,0.0000,-0.5213), Vec3f(-0.5213,0.0000,0.8534) );

		// abrams from the back
		Camera abramsBack(
			Vec3f(-5.081215,-5.600281,-275.260132),
			Vec3f(0.023998,0.000000,0.999716),
			Vec3f(0.999716,0.000000,-0.023998) );

		// abrams from the back + camera turned back
		Camera abramsBackTurned(
			Vec3f(-14.741514,-3.600281,-217.086441),
			Vec3f(-0.181398,0.000000,-0.983413),
			Vec3f(-0.983413,0.000000,0.181398) );

		// bunny, feline, dragon
		Camera bunny(
			Vec3f(7.254675,2.399719,39.409294),
			Vec3f(-0.240041,0.000000,-0.970767),
			Vec3f(-0.970767,0.000000,0.240041) );

		Camera room(
			Vec3f(-58.125217,-205.600281,61.583553),
			Vec3f(0.713451,0.000000,-0.700709),
			Vec3f(-0.700709,0.000000,-0.713451) );

		Camera sponza( Vec3f(443.2726,-248.0000,0.8550), Vec3f(-0.9995,0.0000,-0.0324), Vec3f(-0.0324,0.0000,0.9995) );
		Camera feline( Vec3f(-3.0111,-5.6003,-2.8642), Vec3f(0.8327,0.0000,0.5537), Vec3f(0.5537,0.0000,-0.8327) );
	
		cams["pompei.obj"]=pompei;
		cams["sponza.obj"]=sponza;
		cams["abrams.obj"]=abramsTop;
		cams["lancia.obj"]=abrams;
		cams["bunny.obj"]=bunny;
		cams["feline.obj"]=feline;
		cams["dragon.obj"]=bunny;
		cams["room.obj"]=room;
	}

	std::map<string,Camera>::iterator iter=cams.find(model);
	return iter==cams.end()?Camera():iter->second;
}

template <class Scene>
TreeStats GenImage(int quadLevels,const Scene &scene,const Camera &camera,Image &image,const Options options,uint tasks) {
	switch(quadLevels) {
//	case 0: return GenImage<0>(scene,camera,image,options,tasks);
	case 1: return GenImage<1>(scene,camera,image,options,tasks);
//	case 2: return GenImage<2>(scene,camera,image,options,tasks);
//	case 3: return GenImage<3>(scene,camera,image,options,tasks);
//	case 4: return GenImage<4>(scene,camera,image,options,tasks);
	default: throw Exception("Quad level not supported.");
	}
}

int main(int argc, char **argv)
{
	int resx=512,resy=512;
	float speed=2.0f;
	
	bool fullscreen=0,nonInteractive=0;
	int threads=GetCoresNum();
	const char *modelFile="feline.obj";

	if(argc>2) { resx=atoi(argv[1]); resy=atoi(argv[2]); }
	if(argc>3) { fullscreen=atoi(argv[3])==1; if(atoi(argv[3])>=2) nonInteractive=1; }
	if(argc>4) threads=atoi(argv[4]);
	if(argc>5) modelFile=argv[5];

	printf("Threads/cores: %d/%d\n",threads,GetCoresNum());

	double buildTime;
	const string fileName=string("scenes/")+modelFile;

	buildTime=GetTime();
	TScene<BIHTree<Triangle> >	bihScene (fileName.c_str());
	buildTime=GetTime()-buildTime;
	printf("BIHTree build time: %.2f sec\n",buildTime);
	bihScene.tree.PrintInfo();

	buildTime=GetTime();
	TScene<KDTree>				kdScene (fileName.c_str());
	buildTime=GetTime()-buildTime;
	printf("KDTree build time: %.2f sec\n",buildTime);
	kdScene.tree.PrintInfo();

	Image img(resx,resy);
	Camera cam=GetDefaultCamera(modelFile);;

	uint quadLevels=1;
	double minTime=1.0f/0.0f,maxTime=0.0f;
	bool useKdTree=0;

	if(nonInteractive) {
		for(int n=atoi(argv[3])-1;n>=0;n--)
			if(useKdTree) GenImage(quadLevels,kdScene,cam,img,Options(),threads);
			else GenImage(quadLevels,bihScene,cam,img,Options(),threads);
		img.SaveToFile("out/output.tga");
	}
	else {
		GLWindow out(resx,resy,fullscreen);
		Options options;
	//	options.reflections^=1;

		while(out.PollEvents()) {
			if(out.Key(Key_lctrl)&&out.Key('C')) break;
			if(out.KeyDown('K')) img.SaveToFile("out/output.tga");
			if(out.KeyDown('P')) options.pixDoubling^=1;
			if(out.KeyDown('O')) options.reflections^=1;
			if(out.KeyDown('I')) options.rdtscShader^=1;
			if(out.KeyDown('G')) options.grid^=1;

			if(out.Key('W')) cam.pos+=cam.front*speed;
			if(out.Key('S')) cam.pos-=cam.front*speed;

			if(out.Key('A')) cam.pos-=cam.right*speed;
			if(out.Key('D')) cam.pos+=cam.right*speed;


			if(out.Key('R')) cam.pos-=cam.up*speed;
			if(out.Key('F')) cam.pos+=cam.up*speed;

			if(out.KeyDown('T')) {
				printf("mode: %s\n",useKdTree?"BIH":"KD");
				useKdTree^=1;
			}
			if(out.KeyDown('Y')) {
				printf("splitting %s\n",kdScene.tree.splittingFlag?"off":"on");
				kdScene.tree.splittingFlag^=1;
			}

		//	if(out.KeyDown('0')) { printf("tracing 2x2\n"); quadLevels=0; }
			if(out.KeyDown('1')) { printf("tracing 4x4\n"); quadLevels=1; }
		//	if(out.KeyDown('2')) { printf("tracing 16x4\n"); quadLevels=2; }
		//	if(out.KeyDown('3')) { printf("tracing 64x4\n"); quadLevels=3; }

			if(out.KeyDown('P')) cam.Print();

			{
				int dx=out.KeyDown(Key_space)?out.MouseMove().x:0,dy=0;
				if(out.Key('N')) dx-=20;
				if(out.Key('M')) dx+=20;
				if(out.Key('V')) dy-=20;
				if(out.Key('B')) dy+=20;
				if(dx) {
					Matrix<Vec4f> rotMat=RotateY(dx*0.003f);
					cam.right=rotMat*cam.right; cam.front=rotMat*cam.front;
				}
			//	if(dy) {
			//		Matrix<Vec4f> rotMat(Quat(AxisAngle(cam.right,-dy*0.002f)));
			//		cam.up=rotMat*cam.up; cam.front=rotMat*cam.front;
			//	}
			}
			
			TreeStats stats;
			double time=GetTime();
			stats=useKdTree?	GenImage(quadLevels,kdScene,cam,img,options,threads):
								GenImage(quadLevels,bihScene,cam,img,options,threads);

			stats.tracedRays=resx*resy;
			time=GetTime()-time; minTime=Min(minTime,time);
			maxTime=Max(time,maxTime);

			int lastTicks=0;
			stats.PrintInfo(resx,resy,time*1000.0);

//			scene.Animate();
			out.RenderImage(img);
			out.SwapBuffers();
		}
	}

	printf("Minimum msec/frame: %.4f (%.2f FPS)\n",minTime*1000.0,1.0f/minTime);
	printf("Maximum msec/frame: %.4f (%.2f FPS)\n",maxTime*1000.0,1.0f/maxTime);
	return 0;
}
