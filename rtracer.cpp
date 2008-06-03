#include <iostream>
#include "ray_generator.h"
#include "task_switcher.h"
#include "scene.h"
#include "camera.h"

#include "bihtree.h"

#include "gl_window.h"

using std::cout;
using std::endl;

struct Options {
	Options(ShadingMode sm,bool refl,bool rdtsc) :shading(sm),reflections(refl),rdtscShader(rdtsc) { }
	Options() { reflections=rdtscShader=0; shading=smFlat; }

	ShadingMode shading;
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

				context.options=TracingOptions(options.reflections?1:0,options.shading,options.rdtscShader);
				
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

Vec3f Center(const TriVector &tris) {
	Vec3f center(0,0,0);
	for(int n=0;n<tris.size();n++)
		center+=tris[n].P1()+tris[n].P2()+tris[n].P3();
	center/=float(tris.size()*3);
	return center;
}

void PrintHelp() {
	printf("Synopsis:    rtracer model_file [options]\nOptions:\n\t-res x y   - set rendering resolution [512 512]\n\t");
	printf("-fullscreen\n\t-tofile   - renders to file out/output.tga\n\t-threads n   - set threads number to n\n\t");
	printf("-shading flat|gouraud   - sets shading mode [flat]\n");
	printf("\nExamples:\n\t./rtracer -res 1280 800 abrams.obj\n\t./rtracer pompei.obj -res 800 600 -fullscreen\n\n");
	printf("Interactive control:\n\tA,W,S,D R,F - move the camera\n\tN,M - rotate camera\n\t");
	printf("k - save image to out/output.tga\n\to - toggle reflections\n\tl - toggle lights\n\t");
	printf("j - toggle lights movement\n\tp - print camera position; save camera configuration\n\t");
	printf("c - center camera position in the scene\n\t");
	//	printf("i - toggle scene complexity visualization (green: # visited nodes  red: # intersections)\n\t");
	printf("0,1,2,3 - change tracing mode (on most scenes 0 is slowest,  2,3 is fastest)\n\tesc - exit\n\n");
}

int main(int argc, char **argv)
{
	printf("Unnamed raytracer v0.0666 by nadult\n");
	if(argc>=2&&string("--help")==argv[1]) {
		PrintHelp();
		return 0;
	}
	else printf("type './rtracer --help' to get some help\n");

	CameraConfigs camConfigs;
	try { Loader("scenes/cameras.dat") & camConfigs; } catch(...) { }

	int resx=512,resy=512;
	bool fullscreen=0,nonInteractive=0;
	int threads=GetCoresNum();
	const char *modelFile="feline.obj";
	Options options;

	for(int n=1;n<argc;n++) {
			 if(string("-res")==argv[n]&&n<argc-2) { resx=atoi(argv[n+1]); resy=atoi(argv[n+2]); n+=2; }
		else if(string("-threads")==argv[n]&&n<argc-1) { threads=atoi(argv[n+1]); n+=1; }
		else if(string("-fullscreen")==argv[n]) { fullscreen=1; n+=1; }
		else if(string("-toFile")==argv[n]) { nonInteractive=1; n+=1; }
		else if(string("-shading")==argv[n]&&n<argc-1) { options.shading=string("gouraud")==argv[n+1]?smGouraud:smFlat; n+=1; }
		else modelFile=argv[n];
	}

	printf("Threads/cores: %d/%d\n\n",threads,GetCoresNum());

	double buildTime=GetTime();
	TScene<BIHTree<Triangle> >	scene ((string("scenes/")+modelFile).c_str());
	buildTime=GetTime()-buildTime;
	printf("BIHTree build time: %.2f sec\n",buildTime);
	scene.tree.PrintInfo();

	Image img(resx,resy,16);
	Camera cam;
	if(!camConfigs.GetConfig(string(modelFile),cam))
		cam.pos=Center(scene.tree.objects);
	
	uint quadLevels=2;
	double minTime=1.0f/0.0f,maxTime=0.0f;

	if(nonInteractive) {
		double time=GetTime();
		GenImage(quadLevels,scene,cam,img,Options(),threads);
		time=GetTime()-time;
		minTime=maxTime=time;
		img.SaveToFile("out/output.tga");
	}
	else {
		GLWindow out(resx,resy,fullscreen);
		scene.lightsEnabled=0;
		scene.tree.maxDensity=520.0f * resx * resy;
		bool lightsAnim=0;
		float speed; {
			Vec3p size=scene.tree.pMax-scene.tree.pMin;
			speed=(size.x+size.y+size.z)*0.005f;
		}

		while(out.PollEvents()) {
			if(out.KeyUp(Key_esc)) break;
			if(out.KeyDown('K')) img.SaveToFile("out/output.tga");
			if(out.KeyDown('O')) options.reflections^=1;
			if(out.KeyDown('I')) options.rdtscShader^=1;
			if(out.KeyDown('C')) cam.pos=Center(scene.tree.objects);
			if(out.KeyDown('P')) {
				camConfigs.AddConfig(string(modelFile),cam);
				Saver("scenes/cameras.dat") & camConfigs;
				cam.Print();
			}
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
				int dx=out.Key(Key_space)?out.MouseMove().x:0,dy=0;
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
