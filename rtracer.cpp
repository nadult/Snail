#include "stdafx.h"
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

#ifdef __GNUC__
	#if defined(__i386__)
	static __inline__ unsigned long long Ticks() {
	unsigned long long int x;
		__asm__ volatile (".byte 0x0f, 0x31" : "=A" (x));
		return x;
	}
	#elif defined(__x86_64__)

//	typedef unsigned long long int unsigned long long;

	static __inline__ unsigned long long Ticks() {
		unsigned hi, lo;
		__asm__ __volatile__ ("rdtsc" : "=a"(lo), "=d"(hi));
		return ( (unsigned long long)lo)|( ((unsigned long long)hi)<<32 );
	}
	#endif
#else //msvc, intel
	static INLINE unsigned long long Ticks() {
		u32 ddlow,ddhigh;

		__asm{
			rdtsc
			mov ddlow, eax
			mov ddhigh, edx
		}
		return ((unsigned long long)ddhigh<<32)|(unsigned long long)ddlow;
	}
#endif

template <class Vec>
Vec Reflect(const Vec &ray,const Vec &nrm)
{
	typename Vec::TScalar dot=(nrm|ray);
	return ray-nrm*(dot+dot);
}

int PacketIdGenerator::base[256];



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
//		AddLight(Vec3f(5.696046,-12.600281,0.779701),Vec3f(500,500,500));
		
		startTree=SlowKDTree(objects);
		tree=KDTree(startTree);
	}
	void Animate()
	{
		for(int n=0;n<nLights;n+=1)
			for(int k=n;k<n+1;k++) {
				Vec3f tmp; Convert(lights[k].pos,tmp);
				tmp=RotateY((0.05f+(n/1)*0.01f)*0.2f)*(tmp-Vec3f(-2.0f,0.0f,5.0f))+Vec3f(-2.0f,0.0f,5.0f);
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
	Vec ShadeLight(const Vec &lightColor,const Base &dot,const Base &lightDistSq) const
	{
		Vec out;
		Base mul=RSqrt(lightDistSq);
//		Base spec=dot*dot*dot;
		out = ( 
				 lightColor*dot
//				+lightColor*spec
				)*mul;
		return out;
	}

	template <class Vec,class Group>
	void RayTrace(int packetId,Group &group,const RaySelector<Group::size> &startSelector,Vec *out,int maxRefl
			,bool rdtscShader,bool primary) const
	{
		typedef typename Vec::TScalar base;
		typedef typename Vec::TBool boolv;

		RaySelector<Group::size> sel=startSelector;

		unsigned long long ticks=Ticks();

		for(int q=0;q<Group::size;q++)
			out[q]=Vec(Const<base,0>::Value());

//		__declspec(align(16))
		u32 objId[ScalarInfo<base>::Multiplicity*Group::size] __attribute__((aligned(16)));

		const base maxDist=Const<base,10000>::Value();
		base dst[Group::size];

		int nColTests;
		KDStats stats=tree.stats;
		tree.TraverseOptimized(PacketIdGenerator::Gen(packetId),group,sel,maxDist,NormalOutput(dst,objId),primary);
		nColTests=tree.stats.colTests-stats.colTests;

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

			for(int n=1;n<ScalarInfo<base>::Multiplicity;n++) {
				const Object *objN=obj+objId[n+q*4];
				if(objN!=obj0) {
					Vec newNrm=objN->Normal(colPos[q]);
					nrm[q]=Condition(ScalarInfo<base>::ElementMask(n),newNrm,nrm[q]);
				}
			}

	//		out[q]=Const<base,1>::Value();
	//		out[q].X()=Abs(Const<base,1>::Value()-dst[q]*Const<base,1,50>::Value());
	//		out[q].Y()=Abs(Const<base,1>::Value()-dst[q]*Const<base,1,200>::Value());
	//		out[q].Z()=Abs(Const<base,1>::Value()-dst[q]*Const<base,1,1000>::Value());
//			Vec3q refl=Reflect(group.Dir(q),nrm[q]);
//			out[q].X()=Condition(refl.X()>Const<base,0>::Value(),Const<base,8,12>::Value(),Const<base,2,12>::Value());
//			out[q].Y()=Condition(refl.Y()>Const<base,0>::Value(),Const<base,8,12>::Value(),Const<base,2,12>::Value());
//			out[q].Z()=Condition(refl.Z()>Const<base,0>::Value(),Const<base,8,12>::Value(),Const<base,2,12>::Value());
			out[q]=nrm[q]|group.Dir(q);
//			out[q].X()=Condition(group.Dir(q).X()<Const<floatq,0>::Value(),Const<floatq,0,2>::Value(),Const<floatq,1>::Value());
//			out[q].Y()=Condition(group.Dir(q).Y()<Const<floatq,0>::Value(),Const<floatq,0,2>::Value(),Const<floatq,1>::Value());
//			out[q].Z()=Condition(group.Dir(q).Z()<Const<floatq,0>::Value(),Const<floatq,0,2>::Value(),Const<floatq,1>::Value());
		}

		for(int n=0;n<nLights;n++) {
			const Light &light=lights[n];
			Vec lightPos=light.pos;

			Vec3q fromLight[Group::size];
			base lightDistSq[Group::size];

			for(int i=0;i<sel.Num();i++) {
				int q=sel.Idx(i);

				Vec lightVec=lightPos-colPos[q];
				lightDistSq[q]=lightVec|lightVec;

				base invLightDist=RSqrt(lightDistSq[q]);
				fromLight[q]=-lightVec*invLightDist;
			}

			base tDst[Group::size];
			//__declspec(align(16))
//			u32 lightObjId[ScalarInfo<base>::Multiplicity*Group::size] __attribute__((aligned(16))); {
//				RayGroup<Group::recLevel,1> tGroup(fromLight,&lightPos);
//				tree.TraverseOptimized(PacketIdGenerator::Gen(packetId),tGroup,sel,maxDist,ShadowOutput(tDst));
//			}

			Vec lightColor=light.color;

			for(int i=0;i<sel.Num();i++) {
				int q=sel.Idx(i);
//				boolv mask=Const<SSEMask,0>::Value();
//				__m128i a=_mm_load_si128((__m128i*)(objId+q*4)),b=_mm_load_si128((__m128i*)(lightObjId+q*4));
//				boolv mask=_mm_castsi128_ps(_mm_cmpeq_epi32(a,b));
				base dot=nrm[q]|-fromLight[q]; {
					boolv mask=dot<=Const<base,0>::Value();
					if(ForAll(mask)) {
						if(n==0) accLight[q]=Const<base,0>::Value();
						continue;
					}
					dot=Condition(!mask,dot);
				}

//				boolv mask=tDst[q]*tDst[q]<lightDistSq[q]*Const<SSEReal,999,1000>::Value();
//				if(ForAll(mask)) // wszystkie punkty zasloniete
//					continue;
				

				Vec col=(//Condition(!mask,
						ShadeLight(lightColor,dot,lightDistSq[q]));

				if(n==0) accLight[q]=col;
				else accLight[q]+=col;
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
			RayTrace(PacketIdGenerator::Gen(packetId),tGroup,sel,reflColor,maxRefl-1,0,0);

			for(int i=0;i<sel.Num();i++) {
				int q=sel.Idx(i);
				out[q]=out[q]*Const<base,8,10>::Value()+reflColor[q]*Const<base,2,10>::Value();
			}
		}

		if(nLights) for(int i=0;i<sel.Num();i++) {
			int q=sel[i];
			out[q]=Condition(!tmask[q],out[q]*accLight[q]);
		}
		else for(int i=0;i<sel.Num();i++) {
			int q=sel[i];
			out[q]=Condition(!tmask[q],out[q]);
		}

		ticks=Ticks()-ticks;

		if(rdtscShader) for(int q=0;q<Group::size;q++) {
//			out[q].X()=floatq(sqrt(float(ticks))*0.001f);
//			out[q].Y()=floatq(float(0));
			out[q].Z()=floatq(float(nColTests)*0.005f);
		}
	}

	vector<Object> objects;

	int nLights;
	Light lights[512];

	SlowKDTree startTree;
	KDTree tree;
};

double ticks=0,lastTicks=0,iters=0;

void GenImage(const Scene &scene,const Camera &cam,Image &out,bool pixDoubling,bool reflections,bool rdtscShader)
{
	unsigned long long tick=Ticks();

	float ratio=float(out.width)/float(out.height);

	enum { QuadLevels=2,
			NQuads=1<<(QuadLevels*2), PWidth=2<<QuadLevels, PHeight=2<<QuadLevels };
	bool grid=0;

	assert((out.height%PHeight)==0&&(out.width%PWidth)==0);
	int w=pixDoubling?out.width/2:out.width,h=pixDoubling?out.height/2:out.height;

	Matrix<Vec4f> rotMat(Vec4f(cam.right),Vec4f(cam.up),Vec4f(cam.front),Vec4f(0,0,0,1));
	rotMat=Transpose(rotMat);

	Vec3q orig; Broadcast(cam.pos,orig);
	scene.tree.Prepare();

	RayGenerator rayGen(QuadLevels,w,h,cam.plane_dist);
	PacketIdGenerator::Init();

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
			}
			RayGroup<QuadLevels,1> group(dir,&orig);

			Vec3q rgb[NQuads];
			scene.RayTrace(PacketIdGenerator::Gen(),group,allSelector,rgb,reflections?1:0,rdtscShader,1);

			rayGen.Decompose(rgb,rgb);
			Vec3f trgb[PWidth*PHeight];
			for(int q=0;q<NQuads;q++)
				Convert(Clamp(rgb[q]*Const<floatq,255>::Value(),
						Vec3q(Const<floatq,0>::Value()),
						Vec3q(Const<floatq,255>::Value())),trgb+q*4);

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

	if(pixDoubling) {
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
	Scene scene(modelFile);
	if(!scene.tree.Test()) return 0;
	bticks=SDL_GetTicks()-bticks;
	printf("KD Tree build time: %.2f sec\n",double(bticks)*0.001);

	int full=0,notFull=0;
	for(int n=0;n<scene.tree.objects.size();n++) {
		if(scene.tree.objects[n].fullInNode) full++;
		else notFull++;
	}
	printf("Objects %d:\nFull KDnode objects: %.2f%%\n\n",full+notFull,100.0*double(full)/double(full+notFull));

	Image img(resx,resy);
	Camera cam;
	cam.plane_dist=0.5f;
//	cam.pos=Vec3f(52.423584,158.399719,51.276756);
//	cam.front=Vec3f(0.999916,0.000000,-0.013203);
//	cam.right=Vec3f(-0.013203,0.000000,-0.999916);

	cam.pos=Vec3f(-125.014099,-7.600281,115.258301);
	cam.front=Vec3f(0.907629,0.000000,-0.419782);
	cam.right=Vec3f(-0.419782,0.000000,-0.907629);



//	Matrix<Vec4f> rotMat=RotateY(-102.7f); cam.right=rotMat*cam.right; cam.front=rotMat*cam.front;

	if(nonInteractive) {
		for(int n=atoi(argv[3]);n>=0;n--) GenImage(scene,cam,img,0,0,0);
		img.SaveToFile("output.tga");
	}
	else {
		SDLOutput out(resx,resy,fullscreen);
		bool showTree=0,pixelDoubling=0,refls=0,rdtscSh=0;

		while(out.PollEvents()) {
			if(out.TestKey(SDLK_ESCAPE)) break;
			if(out.TestKey(SDLK_k)) img.SaveToFile("output.tga");
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
			GenImage(scene,cam,img,pixelDoubling,refls,rdtscSh);
			ticks=SDL_GetTicks()-ticks;

			scene.tree.stats.PrintInfo(resx,resy,lastTicks,ticks);

			Vec3f a,b; Convert(scene.tree.pMin,a); Convert(scene.tree.pMax,b);
		//	scene.Animate();
			out.Render(img);
		}
	}

	printf("Average cycles/frame: %.2f mln\n",ticks/iters);
	return 0;
}
