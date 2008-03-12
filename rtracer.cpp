#include "rtracer.h"
#include <SDL/SDL_keysym.h>
#include <SDL/SDL_timer.h>
#include "ray_generator.h"
#include <iostream>

using std::cout;
using std::endl;


Triangle Object::tris[Object::MaxObjs];
Sphere Object::spheres[Object::MaxObjs];
Vec3p Object::bounds[Object::MaxObjs*2];
int Object::nObjs=0;

template <class Vec>
Vec Reflect(const Vec &ray,const Vec &nrm)
{
	typename Vec::TScalar dot=(nrm|ray);
	return ray-nrm*(dot+dot);
}


class Scene
{
public:
	Scene(const char *modelFile)
		:tree(vector<Object>()),startTree(vector<Object>()),nLights(0)
	{
//		objects.push_back(Sphere(Vec3f(-2,0,5)*5.0f,4.04f*5));
//		objects.push_back(Sphere(Vec3f(3,2,10)*5.0f,3*5));
//		objects.push_back(Sphere(Vec3f(0,0,-2.0)*5.0f,0.3f*5));
//		objects.push_back(Sphere(Vec3f(4.1,1.5f,5)*5.0f,1*5));
//		objects.push_back(Sphere(Vec3f(4.1,0.5f,-3.5f)*5.0f,1*5)); 

		srand(1235);

//		for(int n=0;n<2000;n++) {
//			Vec3f pos(FRand()*38.0f-20.0f,FRand()*38.0f-20.0f,FRand()*38.0f-20.0f); pos*=8.0f;
//			for(int n=0;n<20;n++)
//			objects.push_back(
//				Sphere(Vec3f(FRand()*7.80f-4.0f,FRand()*7.80f-4.0f,FRand()*7.80f-4.0f)+pos,
//							FRand()*1.50f));
//		}

	LoadWavefrontObj(modelFile,objects,20.0f);

	//	AddSoftLight(Vec3f(-2,8.0f,0.9f),Vec3f(800,805,805),Vec3f(40,40,40),1,1,1);
	//	AddSoftLight(Vec3f(-30,-20,0),Vec3f(10,0,1020),Vec3f(40,40,40),1,1,1);
	//	AddSoftLight(Vec3f(20,-20,40),Vec3f(100,1000,10),Vec3f(40,40,40),1,1,1);
	//	AddLight(Vec3f(0,-150,0),Vec3f(4000,4000,4000));
		
		startTree=SlowKDTree(objects);
		tree=KDTree(startTree);
	}
	void Animate()
	{
		for(int n=0;n<nLights;n+=1)
			for(int k=n;k<n+1;k++) {
				Vec3f tmp; Convert(lights[k].pos,tmp);
				tmp=RotateY((0.05f+(n/1)*0.01f)*0.2f)*(tmp-Vec3f(0,0,50))+Vec3f(0,0,50);
				Convert(tmp,lights[k].pos);
			}
	}

	void AddLight(Vec3f pos,Vec3f col)
	{
		lights[nLights++]=Light(pos,col);
	}

	void AddSoftLight(Vec3f pos,Vec3f col,Vec3f dens,int dx,int dy,int dz)
	{
		assert(dx>0&&dy>0&&dz>0);
		col/=float(dx*dy*dz);

		for(int x=0;x<dx;x++) for(int y=0;y<dy;y++) for(int z=0;z<dz;z++)
			lights[nLights++]=Light(pos+dens*Vec3f(x/float(dx)-0.5f,y/float(dy)-0.5f,z/float(dz)-0.5f),col);
	}

	template <class Vec,class Base>
	Vec ShadeLight(const Vec &lightColor,const Base &dot,const Base &lightDist) const
	{
		Vec out;
		Base mul=Inv(lightDist*lightDist);
		Base spec=dot*dot; spec=spec*spec;
		out = ( 
				 lightColor*dot
				+Vec3q(lightColor.x,Const<Base,0>(),Const<Base,0>())*spec
				)*mul;
		return out;
	}

	template <class Vec,class Group>
	void RayTrace(Group &group,const RaySelector<Group::size> &startSelector,Vec *out,int maxRefl,bool rdtscShader,bool primary) const
	{
		typedef typename Vec::TScalar base;
		typedef typename Vec::TBool boolv;

		RaySelector<Group::size> sel=startSelector;

		unsigned long long ticks=Ticks();

		for(int q=0;q<Group::size;q++)
			out[q]=Vec(Const<base,0>());

		SSEI32 objId4[Group::size];
		u32 *objId=(u32*)objId4;

		const base maxDist=Const<base,10000>();
		base dst[Group::size];

		int nColTests,nSkips,depth; float ncp;
		KDStats stats=tree.stats;
//		depth=tree.GetDepth(group,sel);
		tree.TraverseOptimized(group,sel,maxDist,NormalOutput(dst,objId4),primary);

		Vec3q nrm[Group::size],colPos[Group::size],reflDir[Group::size],accLight[Group::size];
		boolv tmask[Group::size];

		for(int i=0;i<sel.Num();i++) {
			int q=sel.Idx(i);
			const Object *obj=&tree.objects[0];

			tmask[q]=dst[q]>=maxDist;
			if(ForAll(tmask[q])) { sel.Disable(i--); continue; }

			const Object *obj0=obj+objId[0+q*4];
			colPos[q]=group.Dir(q)*dst[q]+group.Origin(q);
			nrm[q]=obj0->Normal(colPos[q]);

			for(int n=1;n<ScalarInfo<base>::multiplicity;n++) {
				const Object *objN=obj+objId[n+q*4];
				if(objN!=obj0) {
					Vec newNrm=objN->Normal(colPos[q]);
					nrm[q]=Condition(ScalarInfo<base>::ElementMask(n),newNrm,nrm[q]);
				}
			}
	//		accLight[q]=nrm[q]|group.Dir(q);
			accLight[q]=Const<base,0>();

			out[q]=Const<base,1>();
	//		out[q].x=Abs(Const<base,1>()-dst[q]*Const<base,1,50>());
	//		out[q].y=Abs(Const<base,1>()-dst[q]*Const<base,1,200>());
	//		out[q].z=Abs(Const<base,1>()-dst[q]*Const<base,1,1000>());
//			Vec3q refl=Reflect(group.Dir(q),nrm[q]);
//			out[q].x=Condition(refl.x>Const<base,0>(),Const<base,8,12>(),Const<base,2,12>());
//			out[q].y=Condition(refl.y>Const<base,0>(),Const<base,8,12>(),Const<base,2,12>());
//			out[q].z=Condition(refl.z>Const<base,0>(),Const<base,8,12>(),Const<base,2,12>());
			out[q]=nrm[q]|group.Dir(q);
//			out[q].x=Condition(group.Dir(q).x<Const<floatq,0>(),Const<floatq,0,2>(),Const<floatq,1>());
//			out[q].y=Condition(group.Dir(q).y<Const<floatq,0>(),Const<floatq,0,2>(),Const<floatq,1>());
//			out[q].z=Condition(group.Dir(q).z<Const<floatq,0>(),Const<floatq,0,2>(),Const<floatq,1>());
		}

		for(int n=0;n<nLights;n++) {
			const Light &light=lights[n];
			Vec3p lightPos=light.pos;

			Vec3q fromLight[Group::size];
			base lightDist[Group::size];

			for(int i=0;i<sel.Num();i++) {
				int q=sel.Idx(i);

				Vec3q lightVec=(colPos[q]-Vec3q(lightPos));
				lightDist[q]=Sqrt(lightVec|lightVec);
				fromLight[q]=lightVec*Inv(lightDist[q]);
			}

			RaySelector<Group::size> lsel(sel);
			base dot[Group::size];
			for(int i=0;i<lsel.Num();i++) {
				int q=lsel[i];
				dot[q]=nrm[q]|fromLight[q]; {
					boolv mask=dot[q]<=Const<base,0>();
					if(ForAll(mask)) {
						lsel.Disable(i--);
						continue;
					}
					dot[q]=Condition(!mask,dot[q]);
				}
			}

			base tDst[Group::size]; {
				Vec3q lPos(lightPos.x,lightPos.y,lightPos.z);
				RayGroup<Group::recLevel,1> tGroup(fromLight,&lPos);
				tree.TraverseOptimized(tGroup,lsel,Const<base,10000>(),ShadowOutput(tDst),1);
			}


			Vec lightColor=light.color;

			for(int i=0;i<lsel.Num();i++) {
				int q=lsel.Idx(i);

				boolv mask=Abs(tDst[q]-lightDist[q])>Const<base,1,1000>();
				if(ForAll(mask)) // wszystkie punkty zasloniete
					continue;	

				Vec col=Condition(!mask,ShadeLight(lightColor,dot[q],lightDist[q]));
				accLight[q]+=col;
			}
		}

		if(maxRefl>0&&sel.Num()) {
			Vec3q reflColor[Group::size];
			Vec3q reflDir[Group::size];
			for(int i=0;i<sel.Num();i++) {
				int q=sel.Idx(i);
				reflDir[q]=Reflect(group.Dir(q),nrm[q]);
			}

			RayGroup<Group::recLevel,0> tGroup(reflDir,colPos);
			RayTrace(tGroup,sel,reflColor,maxRefl-1,0,0);

			for(int i=0;i<sel.Num();i++) {
				int q=sel.Idx(i);
				out[q]=out[q]*Const<base,8,10>()+reflColor[q]*Const<base,2,10>();
			}
		}

		nColTests=tree.stats.colTests-stats.colTests;
		nSkips=tree.stats.skips-stats.skips;
		ncp=float(tree.stats.nonCoherent-stats.nonCoherent)
			/float(tree.stats.coherent-stats.coherent+tree.stats.nonCoherent-stats.nonCoherent);

		if(nLights) for(int i=0;i<sel.Num();i++) {
			int q=sel[i];
			out[q]=Condition(!tmask[q],out[q]*accLight[q]);
		}
		else for(int i=0;i<sel.Num();i++) {
			int q=sel[i];
			out[q]=Condition(!tmask[q],out[q]);
		}

		ticks=Ticks()-ticks;

		if(rdtscShader) {
			floatq mul(0.0002f/Group::size);
			for(int q=0;q<Group::size;q++) {
			//	out[q].x=floatq(float(ticks))*mul;
			//	out[q].y=floatq(float(ticks))*mul*0.1f;
				out[q].y=floatq(float(nSkips)*0.05f);
				out[q].x=out[q].z=floatq(float(0));//nColTests)*0.001f);
			//	out[q].y=floatq(float(depth)*0.01f);
			}
		}
	}

	vector<Object> objects;

	int nLights;
	Light lights[512];

	SlowKDTree startTree;
	KDTree tree;
};

double ticks=0,lastTicks=0,iters=0;
double minTicks=-1;

template <int QuadLevels>
void GenImage(const Scene &scene,const Camera &cam,Image &out,int pixDoubling,bool reflections,
		bool rdtscShader)
{
	if(pixDoubling==1) {
		GenImage<QuadLevels==0?0:QuadLevels-1>(scene,cam,out,2,reflections,rdtscShader);
		return;
	}

	unsigned long long tick=Ticks();

	float ratio=float(out.width)/float(out.height);

	enum { NQuads=1<<(QuadLevels*2), PWidth=2<<QuadLevels, PHeight=2<<QuadLevels };
	bool grid=0;

	assert((out.height%PHeight)==0&&(out.width%PWidth)==0);
	int w=pixDoubling==2?out.width/2:out.width,h=pixDoubling==2?out.height/2:out.height;

	Matrix<Vec4f> rotMat(Vec4f(cam.right),Vec4f(cam.up),Vec4f(cam.front),Vec4f(0,0,0,1));
	rotMat=Transpose(rotMat);

	Vec3q origin; Broadcast(cam.pos,origin);

	RayGenerator rayGen(QuadLevels,w,h,cam.plane_dist);

	RaySelector<NQuads> allSelector;
	allSelector.SelectAll();

#pragma omp parallel for
	for(int y=0;y<h;y+=PHeight) {
		char *buf[PHeight];
		for(int ty=0;ty<PHeight;ty++)
			buf[ty]=&out.buffer[(y+ty)*out.width*3];

		for(int x=0;x<w;x+=PWidth) {
			Vec3q dir[NQuads];
			rayGen.Generate(PWidth,PHeight,x,y,dir);
			for(int n=0;n<NQuads;n++) {
				Vec3f tmp[4]; Convert(dir[n],tmp);
				for(int k=0;k<4;k++) tmp[k]=rotMat*tmp[k];
				Convert(tmp,dir[n]);
				dir[n]*=RSqrt(dir[n]|dir[n]);
			}
			RayGroup<QuadLevels,1> group(dir,&origin);

			Vec3q rgb[NQuads];
			scene.RayTrace(group,allSelector,rgb,reflections?1:0,rdtscShader,1);

			rayGen.Decompose(rgb,rgb);
			Vec3f trgb[PWidth*PHeight];
			for(int q=0;q<NQuads;q++)
				Convert(VClamp(rgb[q]*Const<floatq,255>(),
						Vec3q(Const<floatq,0>()),
						Vec3q(Const<floatq,255>())),trgb+q*4);

			for(int ty=0;ty<PHeight;ty++) for(int tx=0;tx<PWidth;tx++) {
				*buf[ty]++=(char)trgb[tx+ty*PWidth].x;
				*buf[ty]++=(char)trgb[tx+ty*PWidth].y;
				*buf[ty]++=(char)trgb[tx+ty*PWidth].z;
			}
		}
	}

	if(grid) {
		for(int y=0;y<h;y++) {
			int addX=y%PHeight==0?1:PWidth;
			unsigned char *buf=(unsigned char*)&out.buffer[y*out.width*3];

			for(int x=0;x<w;x+=addX)
				{ buf[x*3+0]=255; buf[x*3+1]=0; buf[x*3+2]=0; }
		}
	}

	if(pixDoubling==2) {
		for(int y=out.height-1;y>=0;y--) for(int x=out.width-1;x>=0;x--) {
			int src=((y/2)*out.width+x/2)*3,dst=(y*out.width+x)*3;
			out.buffer[dst+0]=out.buffer[src+0];
			out.buffer[dst+1]=out.buffer[src+1];
			out.buffer[dst+2]=out.buffer[src+2];
		}
	}

	tick=Ticks()-tick;
	lastTicks=tick/1000000.0;
	ticks+=lastTicks; iters++;
	if(lastTicks<minTicks||minTicks<0) minTicks=lastTicks;
}

#ifdef _DEBUG
	void omp_set_num_threads(int) { }
	int omp_get_max_threads() { return 1; }
	int omp_get_num_procs() { return 1; }
	int omp_get_thread_num() { return 1; }
#endif

int main(int argc, char **argv)
{
/*#ifdef _DEBUG
	int resx=64,resy=64;
	float speed=0.5f;
#else*/
	int resx=512,resy=512;
	float speed=2.0f;
//#endif

#define PRINT_SIZE(cls)		{printf("sizeof(" #cls "): %d\n",sizeof(cls));}
	PRINT_SIZE(SSEVec3) PRINT_SIZE(SSEVec4)
	PRINT_SIZE(SSEPVec3) PRINT_SIZE(SSEPVec4)
	PRINT_SIZE(Object)
	PRINT_SIZE(KDNode)
	PRINT_SIZE(Triangle)
	PRINT_SIZE(Sphere)
#undef PRINT_SIZE
	
	bool fullscreen=0,nonInteractive=0;
	int threads=omp_get_num_procs()*8;
	const char *modelFile="lancia.obj";

	if(argc>2) { resx=atoi(argv[1]); resy=atoi(argv[2]); }
	if(argc>3) { fullscreen=atoi(argv[3])==1; if(atoi(argv[3])>=2) nonInteractive=1; }
	if(argc>4) threads=atoi(argv[4]);
	if(argc>5) modelFile=argv[5];

	omp_set_num_threads(threads);
	printf("Threads/cores: %d/%d\n",omp_get_max_threads(),omp_get_num_procs());

	int bticks=SDL_GetTicks();
	Scene scene((string("scenes/")+modelFile).c_str());
	if(!scene.tree.Test()) return 0;
	bticks=SDL_GetTicks()-bticks;
	printf("KD Tree build time: %.2f sec\n",double(bticks)*0.001);

	int full=0,notFull=0;
	for(int n=0;n<scene.tree.objects.size();n++) {
		if(scene.tree.objects[n].fullInNode) full++;
		else notFull++;
	}
	printf("Objects %d:\nFull KDnode objects: %.2f%%\n\n",full+notFull,100.0*double(full)/double(full+notFull));
	printf("Nodes: %d\n",scene.tree.nodes.size());

	Image img(resx,resy);
	Camera cam;
	cam.plane_dist=0.5f;
	// pompei?
//	cam.pos=Vec3f(52.423584,158.399719,51.276756);
//	cam.front=Vec3f(0.999916,0.000000,-0.013203);
//	cam.right=Vec3f(-0.013203,0.000000,-0.999916);

	// abrams, lancia from the side
	cam.pos=Vec3f(-125.014099,-7.600281,115.258301);
	cam.front=Vec3f(0.907629,0.000000,-0.419782);
	cam.right=Vec3f(-0.419782,0.000000,-0.907629);

	// abrams from the back
//	cam.pos=Vec3f(-5.081215,-5.600281,-275.260132);
//	cam.front=Vec3f(0.023998,0.000000,0.999716);
//	cam.right=Vec3f(0.999716,0.000000,-0.023998);

	// abrams from the back + camera turned back
//	cam.pos=Vec3f(-14.741514,-3.600281,-217.086441);
//	cam.front=Vec3f(-0.181398,0.000000,-0.983413);
//	cam.right=Vec3f(-0.983413,0.000000,0.181398);

	// bunny, feline, dragon
//	cam.pos=Vec3f(7.254675,2.399719,39.409294);
//	cam.front=Vec3f(-0.240041,0.000000,-0.970767);
//	cam.right=Vec3f(-0.970767,0.000000,0.240041);

	// room
//	cam.pos=Vec3f(-58.125217,-205.600281,61.583553);
//	cam.front=Vec3f(0.713451,0.000000,-0.700709);
//	cam.right=Vec3f(-0.700709,0.000000,-0.713451);

//	Matrix<Vec4f> rotMat=RotateY(-102.7f); cam.right=rotMat*cam.right; cam.front=rotMat*cam.front;

	enum { QuadLevels=2 };

	if(nonInteractive) {
		for(int n=atoi(argv[3]);n>=0;n--) GenImage<QuadLevels>(scene,cam,img,0,0,0);
		img.SaveToFile("out/output.tga");
	}
	else {
		SDLOutput out(resx,resy,fullscreen);
		bool showTree=0,pixelDoubling=0,refls=0,rdtscSh=0;

		while(out.PollEvents()) {
			if(out.TestKey(SDLK_ESCAPE)) break;
			if(out.TestKey(SDLK_k)) img.SaveToFile("out/output.tga");
			if(out.TestKey(SDLK_p)) pixelDoubling^=1;
			if(out.TestKey(SDLK_o)) refls^=1;
			if(out.TestKey(SDLK_i)) rdtscSh^=1;

			if(out.TestKey(SDLK_w)) cam.pos+=cam.front*speed;
			if(out.TestKey(SDLK_s)) cam.pos-=cam.front*speed;

			if(out.TestKey(SDLK_a)) cam.pos-=cam.right*speed;
			if(out.TestKey(SDLK_d)) cam.pos+=cam.right*speed;

			if(out.TestKey(SDLK_r)) cam.pos-=cam.up*speed;
			if(out.TestKey(SDLK_f)) cam.pos+=cam.up*speed;

			if(out.TestKey(SDLK_p)) {
				printf("cam.pos=Vec3f(%f,%f,%f);\ncam.front=Vec3f(%f,%f,%f);\ncam.right=Vec3f(%f,%f,%f);\n",
						cam.pos.x,cam.pos.y,cam.pos.z,cam.front.x,cam.front.y,cam.front.z,
						cam.right.x,cam.right.y,cam.right.z);
			}

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
			
			scene.tree.stats.Init();
			int ticks=SDL_GetTicks();
			GenImage<QuadLevels>(scene,cam,img,pixelDoubling,refls,rdtscSh);

			ticks=SDL_GetTicks()-ticks;

			scene.tree.stats.PrintInfo(resx,resy,lastTicks,ticks);

			Vec3f a,b; Convert(scene.tree.pMin,a); Convert(scene.tree.pMax,b);
			scene.Animate();
			out.Render(img);
		}
	}

	printf("Average cycles/frame: %.2f mln\nMinimum cycles/frame: %.2f mln\n",ticks/iters,minTicks);
	return 0;
}
