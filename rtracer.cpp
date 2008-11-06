#include <iostream>
#include <baselib_threads.h>
#include "ray_generator.h"
#include "scene.h"
#include "camera.h"

#include "bih/tree.h"
#include "gl_window.h"
#include "formats/loader.h"
#include "base_scene.h"


Matrix<Vec4f> Inverse(const Matrix<Vec4f> &mat) {
	Matrix<Vec4f> mOut;
	
	const float *M=&mat.x.x;
	float *out=&mOut.x.x;
	
	float t[12],m[16];
	for(int n=0;n<4;n++) {
      m[n+ 0]=M[n*4+0]; m[n+ 4]=M[n*4+1];
      m[n+ 8]=M[n*4+2]; m[n+12]=M[n*4+3];
   }
	t[0 ]=m[10]*m[15]; t[1 ]=m[11]*m[14];
	t[2 ]=m[9 ]*m[15]; t[3 ]=m[11]*m[13];
	t[4 ]=m[9 ]*m[14]; t[5 ]=m[10]*m[13];
	t[6 ]=m[8 ]*m[15]; t[7 ]=m[11]*m[12];
	t[8 ]=m[8 ]*m[14]; t[9 ]=m[10]*m[12];
	t[10]=m[8 ]*m[13]; t[11]=m[9 ]*m[12];

	out[0 ] = t[0 ]*m[5 ]+t[3 ]*m[6 ]+t[4 ]*m[7 ];
	out[0 ]-= t[1 ]*m[5 ]+t[2 ]*m[6 ]+t[5 ]*m[7 ];
	out[1 ] = t[1 ]*m[4 ]+t[6 ]*m[6 ]+t[9 ]*m[7 ];
	out[1 ]-= t[0 ]*m[4 ]+t[7 ]*m[6 ]+t[8 ]*m[7 ];
	out[2 ] = t[2 ]*m[4 ]+t[7 ]*m[5 ]+t[10]*m[7 ];
	out[2 ]-= t[3 ]*m[4 ]+t[6 ]*m[5 ]+t[11]*m[7 ];
	out[3 ] = t[5 ]*m[4 ]+t[8 ]*m[5 ]+t[11]*m[6 ];
	out[3 ]-= t[4 ]*m[4 ]+t[9 ]*m[5 ]+t[10]*m[6 ];
	out[4 ] = t[1 ]*m[1 ]+t[2 ]*m[2 ]+t[5 ]*m[3 ];
	out[4 ]-= t[0 ]*m[1 ]+t[3 ]*m[2 ]+t[4 ]*m[3 ];
	out[5 ] = t[0 ]*m[0 ]+t[7 ]*m[2 ]+t[8 ]*m[3 ];
	out[5 ]-= t[1 ]*m[0 ]+t[6 ]*m[2 ]+t[9 ]*m[3 ];
	out[6 ] = t[3 ]*m[0 ]+t[6 ]*m[1 ]+t[11]*m[3 ];
	out[6 ]-= t[2 ]*m[0 ]+t[7 ]*m[1 ]+t[10]*m[3 ];
	out[7 ] = t[4 ]*m[0 ]+t[9 ]*m[1 ]+t[10]*m[2 ];
	out[7 ]-= t[5 ]*m[0 ]+t[8 ]*m[1 ]+t[11]*m[2 ];

	t[0 ]=m[2 ]*m[7 ]; t[1 ]=m[3 ]*m[6 ];
	t[2 ]=m[1 ]*m[7 ]; t[3 ]=m[3 ]*m[5 ];
	t[4 ]=m[1 ]*m[6 ]; t[5 ]=m[2 ]*m[5 ];
	t[6 ]=m[0 ]*m[7 ]; t[7 ]=m[3 ]*m[4 ];
	t[8 ]=m[0 ]*m[6 ]; t[9 ]=m[2 ]*m[4 ];
	t[10]=m[0 ]*m[5 ]; t[11]=m[1 ]*m[4 ];

	out[8 ] = t[0 ]*m[13]+t[3 ]*m[14]+t[4 ]*m[15];
	out[8 ]-= t[1 ]*m[13]+t[2 ]*m[14]+t[5 ]*m[15];
	out[9 ] = t[1 ]*m[12]+t[6 ]*m[14]+t[9 ]*m[15];
	out[9 ]-= t[0 ]*m[12]+t[7 ]*m[14]+t[8 ]*m[15];
	out[10] = t[2 ]*m[12]+t[7 ]*m[13]+t[10]*m[15];
	out[10]-= t[3 ]*m[12]+t[6 ]*m[13]+t[11]*m[15];
	out[11] = t[5 ]*m[12]+t[8 ]*m[13]+t[11]*m[14];
	out[11]-= t[4 ]*m[12]+t[9 ]*m[13]+t[10]*m[14];
	out[12] = t[2 ]*m[10]+t[5 ]*m[11]+t[1 ]*m[9 ];
	out[12]-= t[4 ]*m[11]+t[0 ]*m[9 ]+t[3 ]*m[10];
	out[13] = t[8 ]*m[11]+t[0 ]*m[8 ]+t[7 ]*m[10];
	out[13]-= t[6 ]*m[10]+t[9 ]*m[11]+t[1 ]*m[8 ];
	out[14] = t[6 ]*m[9 ]+t[11]*m[11]+t[3 ]*m[8 ];
	out[14]-= t[10]*m[11]+t[2 ]*m[8 ]+t[7 ]*m[9 ];
	out[15] = t[10]*m[10]+t[4 ]*m[8 ]+t[9 ]*m[9 ];
	out[15]-= t[8 ]*m[9 ]+t[11]*m[10]+t[5 ]*m[8 ];

	float det=1.0f/(m[0]*out[0]+m[1]*out[1]+m[2]*out[2]+m[3]*out[3]);
	for(int n=0;n<16;n++) out[n]*=det;
   
	return mOut;   
}

int TreeVisMain(TriVector &tris);

int gVals[16]={0,};

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

template <class AccStruct,int QuadLevels>
struct GenImageTask {
	GenImageTask(const AccStruct *tr,const Camera &cam,Image *tOut,const Options &opt,uint tx,uint ty,uint tw,uint th,TreeStats *outSt)
		:tree(tr),camera(cam),out(tOut),options(opt),startX(tx),startY(ty),width(tw),height(th),outStats(outSt) {
		}

	uint startX,startY;
	uint width,height;

	TreeStats *outStats;
	const AccStruct *tree;
	Camera camera;
	Image *out;
	Options options;

	void Work() {
		float ratio=float(out->width)/float(out->height);

		enum { NQuads=1<<(QuadLevels*2), PWidth=2<<QuadLevels, PHeight=2<<QuadLevels };

		Matrix<Vec4f> rotMat(Vec4f(camera.right),Vec4f(camera.up),Vec4f(camera.front),Vec4f(0,0,0,1));

		Vec3q origin; Broadcast(camera.pos,origin);
		RayGenerator rayGen(QuadLevels,out->width,out->height,camera.plane_dist);

		uint pitch=out->width*3;
		u8 *outPtr=(u8*)&out->buffer[startY*pitch+startX*3];
		ShadowCache shadowCache;

	//	for(int ky=0;ky<height;ky+=PHeight*2) for(int kx=0;kx<width;kx+=PWidth*2)
//			for(int y=ky;y<=ky+PHeight;y+=PHeight) { for(int x=kx;x<=kx+PWidth;x+=PWidth) {
				
		for(int y=0;y<height;y+=PHeight) {
			for(int x=0;x<width;x+=PWidth) {
				Vec3q dir[NQuads],idir[NQuads];
				rayGen.Generate(PWidth,PHeight,startX+x,startY+y,dir);

				for(int n=0;n<NQuads;n++) {
					Vec3f tmp[4]; Convert(dir[n],tmp);
					for(int k=0;k<4;k++) tmp[k]=rotMat*tmp[k];
					Convert(tmp,dir[n]);
					dir[n]*=RSqrt(dir[n]|dir[n]);
					dir[n].x+=floatq(0.000000000001f);
					dir[n].y+=floatq(0.000000000001f);
					dir[n].z+=floatq(0.000000000001f);
					idir[n]=VInv(dir[n]);
				}

				TracingContext<RayGroup<NQuads,1,1>,RaySelector<NQuads> > context(RayGroup<NQuads,1,1>(&origin,dir,idir));
				Vec3q *rgb=context.color;

				context.options=TracingOptions(options.reflections?1:0,options.shading,options.rdtscShader);
				
				context.shadowCache=shadowCache;
				RayTrace(*tree,context);
				shadowCache=context.shadowCache;

				outStats->Update(context.stats);

				if(NQuads==1) {
					i32x4 col=ConvColor(rgb[0]);
					const u8 *c=(u8*)&col;

					u8 *p1=outPtr+y*pitch+x*3;
					u8 *p2=p1+pitch;

					p1[ 0]=c[ 0]; p1[ 1]=c[ 1]; p1[ 2]=c[ 2];
					p1[ 3]=c[ 4]; p1[ 4]=c[ 5]; p1[ 5]=c[ 6];
					p2[ 0]=c[ 8]; p2[ 1]=c[ 9]; p2[ 2]=c[10];
					p2[ 3]=c[12]; p2[ 4]=c[13]; p2[ 5]=c[14];
				}
				else {
					rayGen.Decompose(rgb,rgb);

					Vec3q *src=rgb;
					u8 *dst=outPtr+x*3+y*pitch;
					int lineDiff=pitch-PWidth*3;

					for(int ty=0;ty<PHeight;ty++) {
						for(int tx=0;tx<PWidth;tx+=4) {
							i32x4 col=ConvColor(*src++);
							const u8 *c=(u8*)&col;
	
							dst[ 0]=c[ 0]; dst[ 1]=c[ 1]; dst[ 2]=c[ 2];
							dst[ 3]=c[ 4]; dst[ 4]=c[ 5]; dst[ 5]=c[ 6];
							dst[ 6]=c[ 8]; dst[ 7]=c[ 9]; dst[ 8]=c[10];
							dst[ 9]=c[12]; dst[10]=c[13]; dst[11]=c[14];

							dst+=12;
						}
						dst+=lineDiff;
					}
				}
			}
		}
	}
};

template <int QuadLevels,class AccStruct>
TreeStats GenImage(const AccStruct &tree,const Camera &camera,Image &image,const Options options,uint tasks) {
	enum { taskSize=64 };

	uint numTasks=(image.width+taskSize-1)*(image.height+taskSize-1)/(taskSize*taskSize);

	vector<TreeStats> taskStats(numTasks);

	TaskSwitcher<GenImageTask<AccStruct,QuadLevels> > switcher(numTasks);
	uint num=0;
	for(uint y=0;y<image.height;y+=taskSize) for(uint x=0;x<image.width;x+=taskSize) {
		switcher.AddTask(GenImageTask<AccStruct,QuadLevels> (&tree,camera,&image,options,x,y,
					Min((int)taskSize,int(image.width-x)),Min((int)taskSize,int(image.height-y)),
					&taskStats[num]) );
		num++;
	}

	switcher.Work(tasks);

	TreeStats stats;
	for(uint n=0;n<numTasks;n++)
		stats.Update(taskStats[n]);
	return stats;
}

template <class AccStruct>
TreeStats GenImage(int quadLevels,const AccStruct &tree,const Camera &camera,Image &image,const Options options,uint tasks) {
	switch(quadLevels) {
	case 0: return GenImage<0>(tree,camera,image,options,tasks);
	case 1: return GenImage<1>(tree,camera,image,options,tasks);
	case 2: return GenImage<2>(tree,camera,image,options,tasks);
	case 3: return GenImage<3>(tree,camera,image,options,tasks);
//	case 4: return GenImage<4>(tree,camera,image,options,tasks);
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
	printf("-fullscreen\n\t-toFile   - renders to file out/output.tga\n\t-threads n   - set threads number to n\n\t");
	printf("-shading flat|gouraud   - sets shading mode [flat]\n");
	printf("\nExamples:\n\t./rtracer -res 1280 800 abrams.obj\n\t./rtracer pompei.obj -res 800 600 -fullscreen\n\n");
	printf("Interactive control:\n\tA,W,S,D R,F - move the camera\n\tN,M - rotate camera\n\t");
	printf("k - save image to out/output.tga\n\to - toggle reflections\n\tl - toggle lights\n\t");
	printf("j - toggle lights movement\n\tp - print camera position; save camera configuration\n\t");
	printf("c - center camera position in the scene\n\t");
	//	printf("i - toggle scene complexity visualization (green: # visited nodes  red: # intersections)\n\t");
	printf("0,1,2,3 - change tracing mode (on most scenes 0 is slowest,  2,3 is fastest)\n\t");
	printf("F1 - toggle shadow caching\n\t");
	printf("esc - exit\n\n");
}


#include <gfxlib_font.h>

class Font {
	Font() :font("data/fonts/font1.fnt") { }
	
	gfxlib::Font font;
};

typedef bih::Tree<Triangle> StaticTree;
typedef bih::Tree<bih::BIHBox<bih::Tree<Triangle> > > FullTree;

class SceneBuilder {
public:
	struct Object {
		Matrix<Vec4f> preTrans;
		bih::Tree<Triangle> *tree;
		BBox bBox;
	};
	
	struct Instance {
		Matrix<Vec4f> trans;
		u32 objId;
	};

	// box includes preTrans (it can be more optimized than just tree->BBox()*preTrans)
	void AddObject(bih::Tree<Triangle> *tree,const Matrix<Vec4f> &preTrans,const BBox &box) {
		Object newObj;
		newObj.tree=tree;
		newObj.bBox=box;
		newObj.preTrans=preTrans;
		objects.push_back(newObj);
	}

	void AddInstance(int objId,const Matrix<Vec4f> &trans) {
		Instance newInst;
		newInst.trans=trans;
		newInst.objId=objId;
		instances.push_back(newInst);
	}

	typedef bih::BIHBox<bih::Tree<Triangle> > Elem;

	vector<Elem,AlignedAllocator<Elem> > ExtractElements() const {
		vector<Elem,AlignedAllocator<Elem> > elements;

		for(int n=0;n<instances.size();n++) {
			const Instance &inst=instances[n];
			const Object &obj=objects[inst.objId];
			elements.push_back(Elem(obj.tree,obj.preTrans*inst.trans,obj.bBox));
		}

		return elements;
	}
	
	vector<Object> objects;
	vector<Instance> instances;
};


int main(int argc, char **argv) {
	printf("Unnamed raytracer v0.07 by nadult\n");
	if(argc>=2&&string("--help")==argv[1]) {
		PrintHelp();
		return 0;
	}
	else printf("type './rtracer --help' to get some help\n");

	CameraConfigs camConfigs;
	try { Loader("scenes/cameras.dat") & camConfigs; } catch(...) { }

	int resx=800,resy=600;
	bool fullscreen=0,nonInteractive=0;
	int threads=4;
	const char *modelFile="abrams.obj";
	Options options;
	bool treeVisMode=0;

	for(int n=1;n<argc;n++) {
			 if(string("-res")==argv[n]&&n<argc-2) { resx=atoi(argv[n+1]); resy=atoi(argv[n+2]); n+=2; }
		else if(string("-threads")==argv[n]&&n<argc-1) { threads=atoi(argv[n+1]); n+=1; }
		else if(string("-fullscreen")==argv[n]) { fullscreen=1; }
		else if(string("-toFile")==argv[n]) { nonInteractive=1; }
		else if(string("-shading")==argv[n]&&n<argc-1) { options.shading=string("gouraud")==argv[n+1]?smGouraud:smFlat; n+=1; }
		else if(string("-treevis")==argv[n]) treeVisMode=1;
		else modelFile=argv[n];
	}

	printf("Threads/cores: %d/%d\n\n",threads,4);

	TriVector tris;
	ShadingDataVec shadingData;
	printf("Loading...\n");
	BaseScene baseScene; {
		baseScene.LoadWavefrontObj(string("scenes/")+modelFile);
		tris=baseScene.ToTriVector();
	/*	if(baseScene.objects.size()==1) {
			BaseScene::Object obj=baseScene.objects[0];
			baseScene.objects[0]=baseScene.objects.back();
			baseScene.objects.pop_back();
				
			cout << "Splitting..." << '\n';
			obj.BreakToElements(baseScene.objects);
		} */
	//	baseScene.SaveWavefrontObj("out/splitted.obj");
		baseScene.Optimize();
	}

	if(treeVisMode) { TreeVisMain(tris); return 0; }

	printf("Building..\n");
	SceneBuilder builder;
	for(int n=0;n<baseScene.objects.size();n++) {
		const BaseScene::Object &obj=baseScene.objects[n];
		builder.AddObject(new StaticTree(obj.ToTriVector()),obj.GetTrans(),obj.GetBBox());
		builder.AddInstance(n,Identity<>());
	}
	printf("Done building\n");

	Image img(resx,resy,16);
	Camera cam;
	if(!camConfigs.GetConfig(string(modelFile),cam))
		cam.pos=Center(tris);

	uint quadLevels=1;
	double minTime=1.0f/0.0f,maxTime=0.0f;
	
	for(int n=0;n<4;n++)
		gVals[n]=1;

	StaticTree staticTree(tris);

	if(nonInteractive) {
		double time=GetTime();
		GenImage(quadLevels,staticTree,cam,img,options,threads);
		time=GetTime()-time;
		minTime=maxTime=time;
		img.SaveToFile("out/output.tga");
	}
	else {
		GLWindow out(resx,resy,fullscreen);
		bool lightsAnim=0;
		float speed; {
			Vec3p size=baseScene.GetBBox().Size();
			speed=(size.x+size.y+size.z)*0.005f;
		}

		while(out.PollEvents()) {
			if(out.KeyUp(Key_esc)) break;
			if(out.KeyDown('K')) img.SaveToFile("out/output.tga");
			if(out.KeyDown('O')) options.reflections^=1;
			if(out.KeyDown('I')) options.rdtscShader^=1;
			if(out.KeyDown('C')) cam.pos=Center(tris);
			if(out.KeyDown('P')) {
				camConfigs.AddConfig(string(modelFile),cam);
				Saver("scenes/cameras.dat") & camConfigs;
				cam.Print();
			}
		//	if(out.KeyDown('L')) { printf("Lights %s\n",scene.lightsEnabled?"disabled":"enabled"); scene.lightsEnabled^=1; }
			if(out.KeyDown('J')) { printf("Lights animation %s\n",lightsAnim?"disabled":"enabled"); lightsAnim^=1; }

			{
				float tspeed=speed*(out.Key(Key_lshift)?3.0f:1.0f);
				if(out.Key('W')) cam.pos+=cam.front*tspeed;
				if(out.Key('S')) cam.pos-=cam.front*tspeed;
				if(out.Key('A')) cam.pos-=cam.right*tspeed;
				if(out.Key('D')) cam.pos+=cam.right*tspeed;
				if(out.Key('R')) cam.pos-=cam.up*tspeed;
				if(out.Key('F')) cam.pos+=cam.up*tspeed;
			}

		//	if(out.KeyDown('Y')) { printf("splitting %s\n",scene.tree.split?"off":"on"); scene.tree.split^=1; }

			if(out.Key(Key_lctrl)) {
		//		if(out.KeyDown('1')&&scene.lights.size()>=1) scene.lights[0].pos=cam.pos;
		//		if(out.KeyDown('2')&&scene.lights.size()>=2) scene.lights[1].pos=cam.pos;
		//		if(out.KeyDown('3')&&scene.lights.size()>=3) scene.lights[2].pos=cam.pos;
			}
			else {
				if(out.KeyDown('0')) { printf("tracing 2x2\n"); quadLevels=0; }
				if(out.KeyDown('1')) { printf("tracing 4x4\n"); quadLevels=1; }
				if(out.KeyDown('2')) { printf("tracing 16x4\n"); quadLevels=2; }
				if(out.KeyDown('3')) { printf("tracing 64x4\n"); quadLevels=3; }
			}

			if(out.KeyDown(Key_f1)) { gVals[0]^=1; printf("Val 1 %s\n",gVals[0]?"on":"off"); }
			if(out.KeyDown(Key_f2)) { gVals[1]^=1; printf("Val 2 %s\n",gVals[1]?"on":"off"); }
			if(out.KeyDown(Key_f3)) { gVals[2]^=1; printf("Val 3 %s\n",gVals[2]?"on":"off"); }
			if(out.KeyDown(Key_f4)) { gVals[3]^=1; printf("Val 4 %s\n",gVals[3]?"on":"off"); }


			{
				int dx=out.Key(Key_space)?out.MouseMove().x:0,dy=0;
				if(out.Key('N')) dx-=20;
				if(out.Key('M')) dx+=20;
				if(out.Key('V')) dy-=20;
				if(out.Key('B')) dy+=20;
				if(dx) {
					Matrix<Vec4f> rotMat=RotateY(-dx*0.003f);
					cam.right=rotMat*cam.right; cam.front=rotMat*cam.front;
				}
			//	if(dy) {
			//		Matrix<Vec4f> rotMat(Quat(AxisAngle(cam.right,-dy*0.002f)));
			//		cam.up=rotMat*cam.up; cam.front=rotMat*cam.front;
			//	}
			}
	
			double time=GetTime();
		
			double buildTime=GetTime();

			FullTree tree(builder.ExtractElements());

			buildTime=GetTime()-buildTime;
			
			TreeStats stats;
			if(gVals[0]) stats=GenImage(quadLevels,staticTree,cam,img,options,threads);
			else stats=GenImage(quadLevels,tree,cam,img,options,threads);

			out.RenderImage(img);
			out.SwapBuffers();

			time=GetTime()-time; minTime=Min(minTime,time);
			maxTime=Max(time,maxTime);

			stats.PrintInfo(resx,resy,time*1000.0,buildTime*1000.0);

		//	if(lightsAnim) scene.Animate();
		}
	}

	printf("Minimum msec/frame: %.4f (%.2f FPS)\n",minTime*1000.0,1.0f/minTime);
	printf("Maximum msec/frame: %.4f (%.2f FPS)\n",maxTime*1000.0,1.0f/maxTime);

	return 0;
}
