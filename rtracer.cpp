#include <map>
#include <iostream>
#include <memory.h>

#include "ray_generator.h"
#include "task_switcher.h"
#include "scene.h"
#include "camera.h"

#include "bihtree.h"

#include "gl_window.h"

using std::cout;
using std::endl;

struct Options {
	Options(bool refl,bool rdtsc) :reflections(refl),rdtscShader(rdtsc) { }
	Options() { memset(this,0,sizeof(Options)); }

	bool reflections,rdtscShader;
};

inline i32x4 ConvColor(const Vec3q &rgb) {
	i32x4 tr=Trunc(Clamp(rgb.x*255.0f,floatq(0.0f),floatq(255.0f)));
	i32x4 tg=Trunc(Clamp(rgb.y*255.0f,floatq(0.0f),floatq(255.0f)));
	i32x4 tb=Trunc(Clamp(rgb.z*255.0f,floatq(0.0f),floatq(255.0f)));

	__m128i c1,c2,c3,c4;
	c1=_mm_packs_epi32(tr.m,tr.m);
	c2=_mm_packs_epi32(tg.m,tg.m);
	c3=_mm_packs_epi32(tb.m,tb.m);
	c1=_mm_packus_epi16(c1,c1);
	c2=_mm_packus_epi16(c2,c2);
	c3=_mm_packus_epi16(c3,c3);

	__m128i c12=_mm_unpacklo_epi8(c1,c2);
	__m128i c33=_mm_unpacklo_epi8(c3,c3);
	return _mm_unpacklo_epi16(c12,c33);
}

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
				Vec3q dir[NQuads];
				rayGen.Generate(PWidth,PHeight,startX+x,startY+y,dir);

				for(int n=0;n<NQuads;n++) {
					Vec3f tmp[4]; Convert(dir[n],tmp);
					for(int k=0;k<4;k++) tmp[k]=rotMat*tmp[k];
					Convert(tmp,dir[n]);
					dir[n]*=RSqrt(dir[n]|dir[n]);
				}

				TracingContext<Scene,RayGroup<NQuads,1,0>,RaySelector<NQuads> >
					context(*scene,RayGroup<NQuads,1,0>(dir,&origin,0));
				Vec3q *rgb=context.color;

				context.options=TracingOptions(options.reflections?1:0,options.rdtscShader);
				
				scene->RayTracePrimary(context);
				outStats->Update(context.stats);

				if(NQuads==1) {
					i32x4 col=ConvColor(rgb[0]);
					u8 *c=(u8*)&col; u32 *ic=(u32*)&col;

					u8 *p1=outPtr+y*pitch+x*3;
					u8 *p2=p1+pitch;

					*(int*)(p1+0)=col[0];
					p1[ 3]=c[ 4]; p1[ 4]=c[ 5]; p1[ 5]=c[ 6];
					*(int*)(p2+0)=col[2];
					p2[ 3]=c[12]; p2[ 4]=c[13]; p2[ 5]=c[14];
				}
				else {
					rayGen.Decompose(rgb,rgb);

					Vec3q *src=rgb;
					u8 *dst=outPtr+x*3+y*pitch;
					int lineDiff=pitch-PWidth*3+12;

					for(int ty=0;ty<PHeight;ty++) {
						for(int tx=0;tx<PWidth-4;tx+=4) {
							i32x4 col=ConvColor(*src++);
							u8 *c=(u8*)&col;
							*(int*)(dst+0)=col[0];
							*(int*)(dst+3)=col[1];
							*(int*)(dst+6)=col[2];
							*(int*)(dst+9)=col[3];
							dst+=12;
						}
						{
							i32x4 col=ConvColor(*src++);
							u8 *c=(u8*)&col;
							*(int*)(dst+0)=col[0];
							*(int*)(dst+3)=col[1];
							*(int*)(dst+6)=col[2];
							dst[ 9]=c[12]; dst[10]=c[13]; dst[11]=c[14];
							dst+=lineDiff;
						}
					}
				}
			}
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

Camera GetDefaultCamera(string model) {
	static bool initialized=0;
	static std::map< string, Camera> cams;
	if(!initialized) {
		initialized=1;
		Camera pompei(
			Vec3f(52.423584,158.399719,51.276756),
			Vec3f(0.999916,0.000000,-0.013203),
			Vec3f(-0.013203,0.000000,-0.999916));

		Camera abrams(
			Vec3f(-125.014099,-7.600281,115.258301),
			Vec3f(0.907629,0.000000,-0.419782),
			Vec3f(-0.419782,0.000000,-0.907629));

		Camera abramsTop( Vec3f(-118.3508,-93.6003,86.6569), Vec3f(0.8508,0.0000,-0.5254), Vec3f(-0.5254,0.0000,-0.8508) );
		Camera abramsLeak( Vec3f(102.7247,-31.6003,-76.6981), Vec3f(-0.8534,0.0000,-0.5213), Vec3f(-0.5213,0.0000,0.8534) );

		Camera abramsBack(
			Vec3f(-5.081215,-5.600281,-275.260132),
			Vec3f(0.023998,0.000000,0.999716),
			Vec3f(0.999716,0.000000,-0.023998) );

		Camera abramsBadCase(
				Vec3f(-23.2355,-77.6003,-2.7983),
				Vec3f(0.6772,0.0000,0.7357),
				Vec3f(0.7357,0.0000,-0.6772) );

		Camera abramsBackTurned(
			Vec3f(-14.741514,-3.600281,-217.086441),
			Vec3f(-0.181398,0.000000,-0.983413),
			Vec3f(-0.983413,0.000000,0.181398) );

		Camera bunny(
			Vec3f(7.254675,2.399719,39.409294),
			Vec3f(-0.240041,0.000000,-0.970767),
			Vec3f(-0.970767,0.000000,0.240041) );

		Camera room(
			Vec3f(-58.125217,-205.600281,61.583553),
			Vec3f(0.713451,0.000000,-0.700709),
			Vec3f(-0.700709,0.000000,-0.713451) );

		Camera sponza(Vec3f(443.2726,-248.0000,0.8550),Vec3f(-0.9995,0.0000,-0.0324),Vec3f(-0.0324,0.0000,0.9995));
		Camera sponzaBadCase(Vec3f(-353.3055,-326.0000,147.0938),Vec3f(0.0815,0.0000,-0.9967),Vec3f(-0.9967,0.0000,-0.0815));
			
		Camera feline( Vec3f(-3.0111,-5.6003,-2.8642), Vec3f(0.8327,0.0000,0.5537), Vec3f(0.5537,0.0000,-0.8327) );
		Camera box(Vec3f(-12.7319,0.0000,-26.7225),Vec3f(0.3523,0.0000,0.9359),Vec3f(0.9359,0.0000,-0.3523));

		Camera sponzaBug(Vec3f(-281.2263,-104.0000,240.5235),Vec3f(0.8898,0.0000,0.4565),Vec3f(0.4565,0.0000,-0.8898));
		Camera abrams2(Vec3f(-119.3219,-73.6003,86.1858),Vec3f(0.8808,0.0000,-0.4735),Vec3f(-0.4735,0.0000,-0.8808));

		cams["pompei.obj"]=pompei;
		cams["sponza.obj"]=sponza;
		cams["abrams.obj"]=abrams2;
		cams["lancia.obj"]=abrams;
		cams["bunny.obj"]=bunny;
		cams["feline.obj"]=feline;
		cams["dragon.obj"]=bunny;
		cams["room.obj"]=room;
		cams["box.obj"]=box;
	}

	std::map<string,Camera>::iterator iter=cams.find(model);
	return iter==cams.end()?Camera():iter->second;
}

template <class Scene>
TreeStats GenImage(int quadLevels,const Scene &scene,const Camera &camera,Image &image,const Options options,uint tasks) {
	switch(quadLevels) {
	case 0: return GenImage<0>(scene,camera,image,options,tasks);
	case 1: return GenImage<1>(scene,camera,image,options,tasks);
	case 2: return GenImage<2>(scene,camera,image,options,tasks);
//	case 3: return GenImage<3>(scene,camera,image,options,tasks);
//	case 4: return GenImage<4>(scene,camera,image,options,tasks);
	default: throw Exception("Quad level not supported.");
	}
}

int main(int argc, char **argv)
{
	if(argc>=2&&string("--help")==argv[1]) {
		printf("Unnamed raytracer v0.0666 by nadult\n\n");
		printf("Usage:\n\trtracer resx resy mode threads scene\n\tmode: 0: windowed  1: fullscreen  2: render to output.tga\n");
		printf("\nExamples:\n\t./rtracer 1280 800 0 2 abrams.obj\n\t./rtracer 800 600 1 2 pompei.obj\n\n");
		printf("Interactive control:\n\tA,W,S,D R,F - move the camera\n\tN,M - rotate camera\n\t");
		printf("k - save image to out/output.tga\n\to - toggle reflections\n\tl - toggle lights\n\t");
		printf("j - toggle lights movement\n\tp - print camera position\n\t");
	//	printf("i - toggle scene complexity visualization (green: # visited nodes  red: # intersections)\n\t");
		printf("0,1,2,3 - change tracing mode (on most scenes 0 is slowest,  2,3 is fastest)\n\tctrl+c - exit\n\n");

		return 0;
	}

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
	TScene<BIHTree<Triangle> >	scene (fileName.c_str());
	buildTime=GetTime()-buildTime;
	printf("BIHTree build time: %.2f sec\n",buildTime);
	scene.tree.PrintInfo();

	Image img(resx,resy,16);
	Camera cam=GetDefaultCamera(modelFile);;
	
	uint quadLevels=2;
	double minTime=1.0f/0.0f,maxTime=0.0f;

	if(nonInteractive) {
		for(int n=atoi(argv[3])-1;n>0;n--) {
			double time=GetTime();
			GenImage(quadLevels,scene,cam,img,Options(),threads);
			time=GetTime()-time;
			minTime=Min(minTime,time);
			maxTime=Max(maxTime,time);
		}
		img.SaveToFile("out/output.tga");
	}
	else {
		GLWindow out(resx,resy,fullscreen);
		Options options;
		scene.lightsEnabled=0;
		scene.tree.maxDensity=520.0f * resx * resy;
		bool lightsAnim=0;

		while(out.PollEvents()) {
			if(out.Key(Key_lctrl)&&out.Key('C')) break;
			if(out.KeyDown('K')) img.SaveToFile("out/output.tga");
			if(out.KeyDown('O')) options.reflections^=1;
	//		if(out.KeyDown('I')) options.rdtscShader^=1;
			if(out.KeyDown('P')) cam.Print();
			if(out.KeyDown('L')) { printf("Lights %s\n",scene.lightsEnabled?"disabled":"enabled"); scene.lightsEnabled^=1; }
			if(out.KeyDown('J')) { printf("Lights animation %s\n",lightsAnim?"disabled":"enabled"); lightsAnim^=1; }

			if(out.Key('W')) cam.pos+=cam.front*speed;
			if(out.Key('S')) cam.pos-=cam.front*speed;
			if(out.Key('A')) cam.pos-=cam.right*speed;
			if(out.Key('D')) cam.pos+=cam.right*speed;
			if(out.Key('R')) cam.pos-=cam.up*speed;
			if(out.Key('F')) cam.pos+=cam.up*speed;

		//	if(out.KeyDown('Y')) { printf("splitting %s\n",scene.tree.split?"off":"on"); scene.tree.split^=1; }
		//	if(out.KeyDown('[')) { scene.tree.maxDensity/=2.0f; printf("maxdensity: %.0f\n",scene.tree.maxDensity); }
		//	if(out.KeyDown(']')) { scene.tree.maxDensity*=2.0f; printf("maxdensity: %.0f\n",scene.tree.maxDensity); }

			if(out.KeyDown('0')) { printf("tracing 2x2\n"); quadLevels=0; }
			if(out.KeyDown('1')) { printf("tracing 4x4\n"); quadLevels=1; }
			if(out.KeyDown('2')) { printf("tracing 16x4\n"); quadLevels=2; }
		//	if(out.KeyDown('3')) { printf("tracing 64x4\n"); quadLevels=3; }


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
			
			scene.tree.pattern.Init(scene.tree.nodes.size(),resx);

			TreeStats stats;
			double time=GetTime();
			stats=GenImage(quadLevels,scene,cam,img,options,threads);

			out.RenderImage(img);
			out.SwapBuffers();

			time=GetTime()-time; minTime=Min(minTime,time);
			maxTime=Max(time,maxTime);

			stats.PrintInfo(resx,resy,time*1000.0);
			scene.tree.pattern.Draw(img);

			if(lightsAnim) scene.Animate();
		}
	}

	printf("Minimum msec/frame: %.4f (%.2f FPS)\n",minTime*1000.0,1.0f/minTime);
	printf("Maximum msec/frame: %.4f (%.2f FPS)\n",maxTime*1000.0,1.0f/maxTime);
	return 0;
}
