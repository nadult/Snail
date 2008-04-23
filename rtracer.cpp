#include "rtracer.h"
#include <SDL/SDL_keysym.h>
#include "ray_generator.h"
#include <iostream>
#include "bihtree.h"
#include "task_switcher.h"
#include "scene.h"
#include <map>


/*
enum ParameterType {
	pSurfaceNormal,
	pRayDirection,
	pDistance,
	pRayOrigin,
	pPosition,
	pColor,
};

template <class a,class b> class ReflectT { };
template <class lvalue,class rvalue> class SetT { };

template <int ttype> struct Parameter {
	enum { type=ttype };

	template <class r>
	SetT<Parameter<type>,r> operator=(r) { }
};


typedef Parameter<pSurfaceNormal> SurfaceNormalT;
typedef Parameter<pRayDirection> RayDirectionT;
typedef Parameter<pColor> ColorT;

ColorT color;
SurfaceNormalT surfaceNormal;
RayDirectionT rayDirection;


template <int a,int b>
ReflectT<Parameter<a> ,Parameter<b> > Reflect(const Parameter<a>&,const Parameter<b>&) { }


float func() { return 0; }

typedef typeof(

	color = Reflect( surfaceNormal, rayDirection)

	) ReflShader;

template <class T> class Print { Print() { } };

Print<ReflShader> a; void t() { a.x=0; }
*/


using std::cout;
using std::endl;
typedef TScene< KDTree > Scene;

struct Options {
	Options(bool pixD,bool gd,bool refl,bool rdtsc) :pixDoubling(pixD),grid(gd),reflections(refl),rdtscShader(rdtsc) { }
	Options() { memset(this,0,sizeof(Options)); }

	bool pixDoubling,grid;
	bool reflections,rdtscShader;
};


template <int QuadLevels>
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
				Vec3q dir[NQuads];
				rayGen.Generate(PWidth,PHeight,startX+x,startY+y,dir);

				for(int n=0;n<NQuads;n++) {
					Vec3f tmp[4]; Convert(dir[n],tmp);
					for(int k=0;k<4;k++) tmp[k]=rotMat*tmp[k];
					Convert(tmp,dir[n]);
					dir[n]*=RSqrt(dir[n]|dir[n]);
				}

				RayStore<NQuads,1> rayStore(dir,&origin);
				TracingContext<Scene,RayGroup<QuadLevels,1>,Vec3q,i32x4> context(*scene,rayStore);
				Vec3q *rgb=context.color;

				context.options=TracingOptions(options.reflections?1:0,options.rdtscShader);
				
				scene->RayTrace(context);
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


template <int QuadLevels>
TreeStats GenImage(const Scene &scene,const Camera &camera,Image &image,const Options options,uint tasks) {
	enum { taskSize=64 };

	uint numTasks=(image.width+taskSize-1)*(image.height+taskSize-1)/(taskSize*taskSize);

	vector<TreeStats> taskStats(numTasks);

	TaskSwitcher<GenImageTask<QuadLevels> > switcher(numTasks);
	uint num=0;
	for(uint y=0;y<image.height;y+=taskSize) for(uint x=0;x<image.width;x+=taskSize) {
		switcher.AddTask(GenImageTask<QuadLevels> (&scene,camera,&image,options,x,y,
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

		cams["pompei.obj"]=pompei;
		cams["sponza.obj"]=sponza;
		cams["abrams.obj"]=abramsTop;
		cams["lancia.obj"]=abrams;
		cams["bunny.obj"]=bunny;
		cams["feline.obj"]=bunny;
		cams["dragon.obj"]=bunny;
		cams["room.obj"]=room;
	}

	std::map<string,Camera>::iterator iter=cams.find(model);
	return iter==cams.end()?Camera():iter->second;
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

	double buildTime=GetTime();
	Scene scene((string("scenes/")+modelFile).c_str());
	buildTime=GetTime()-buildTime;
	printf("Build time: %.2f sec\n",buildTime);

	scene.tree.PrintInfo();

	Image img(resx,resy);
	Camera cam=GetDefaultCamera(modelFile);;

	enum { QuadLevels=1 };
	double minTime=1.0f/0.0f,maxTime=0.0f;

	if(nonInteractive) {
		for(int n=atoi(argv[3])-1;n>=0;n--) GenImage<QuadLevels>(scene,cam,img,Options(),threads);
		img.SaveToFile("out/output.tga");
	}
	else {
		SDLOutput out(resx,resy,fullscreen);
		Options options;

		while(out.PollEvents()) {
			if(out.TestKey(SDLK_ESCAPE)) break;
			if(out.TestKey(SDLK_k)) img.SaveToFile("out/output.tga");
			if(out.TestKey(SDLK_p)) options.pixDoubling^=1;
			if(out.TestKey(SDLK_o)) options.reflections^=1;
			if(out.TestKey(SDLK_i)) options.rdtscShader^=1;
			if(out.TestKey(SDLK_g)) options.grid^=1;

			if(out.TestKey(SDLK_w)) cam.pos+=cam.front*speed;
			if(out.TestKey(SDLK_s)) cam.pos-=cam.front*speed;

			if(out.TestKey(SDLK_a)) cam.pos-=cam.right*speed;
			if(out.TestKey(SDLK_d)) cam.pos+=cam.right*speed;

			if(out.TestKey(SDLK_r)) cam.pos-=cam.up*speed;
			if(out.TestKey(SDLK_f)) cam.pos+=cam.up*speed;

			if(out.TestKey(SDLK_p)) cam.Print();

			{
				int dx=out.TestKey(SDLK_SPACE)?out.MouseDX():0;
				if(out.TestKey(SDLK_n)) dx-=20;
				if(out.TestKey(SDLK_m)) dx+=20;
				if(dx) {
					Matrix<Vec4f> rotMat=RotateY(dx*0.003f);
					cam.right=rotMat*cam.right; cam.front=rotMat*cam.front;
				}
				//if(out.MouseDY()) {
				//	Matrix<Vec4f> rotMat(Quat(AxisAngle(cam.right,-out.MouseDY()*0.002f)));
				//	cam.up=rotMat*cam.up; cam.front=rotMat*cam.front;
				//}
			}
			
			TreeStats stats;
			double time=GetTime();
			stats=GenImage<QuadLevels>(scene,cam,img,options,threads);
			time=GetTime()-time; minTime=Min(minTime,time);
			maxTime=Max(time,maxTime);

			int lastTicks=0;
			stats.PrintInfo(resx,resy,time*1000.0);

			Vec3f a,b; Convert(scene.tree.pMin,a); Convert(scene.tree.pMax,b);
		//	scene.Animate();
			out.Render(img);
		}
	}

	printf("Minimum msec/frame: %.4f (%.2f FPS)\n",minTime*1000.0,1.0f/minTime);
	printf("Maximum msec/frame: %.4f (%.2f FPS)\n",maxTime*1000.0,1.0f/maxTime);
	return 0;
}
