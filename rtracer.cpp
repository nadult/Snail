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
	Scene()
		:tree(vector<Object>()),startTree(vector<Object>()),nLights(0)
	{
		objects.push_back(Sphere(Vec3f(-2,0,5)*5.0f,4.04f*5));
		objects.push_back(Sphere(Vec3f(3,2,10)*5.0f,3*5));
		objects.push_back(Sphere(Vec3f(0,0,-2.0)*5.0f,0.3f*5));
		objects.push_back(Sphere(Vec3f(4.1,1.5f,5)*5.0f,1*5));
		objects.push_back(Sphere(Vec3f(4.1,0.5f,-3.5f)*5.0f,1*5)); 

		srand(1235);
		for(int n=0;n<100;n++)
			objects.push_back(
				Sphere(Vec3f(FRand()*38.0f-20.0f,FRand()*38.0f-20.0f,FRand()*38.0f-20.0f)*8.0f,
							FRand()*15.50f));

	//	objects.push_back(Triangle(Vec3f(1000,-30,1000),Vec3f(1000,-30,-1000),Vec3f(-100,-30,1000)));
	//	objects.push_back(Triangle(Vec3f(-1000,-30,1000),Vec3f(1000,-30,-1000),Vec3f(-100,-31,-1000)));

	//	for(int n=0;n<64;n++) {
	//		objects.push_back(Triangle(	Vec3f(0,								-2,		0),
	//									Vec3f(cos(n*6.2831f/64.0f)*10,			-10,		sin(n*6.2831f/64.0f)*10),
	//									Vec3f(cos(((n+1)&63)*6.2831f/64.0f)*10,	-10,		sin(((n+1)&63)*6.2831f/64.0f)*10) ));
	//	}
		
	//	objects.push_back(Triangle(Vec3f(0,10,0),Vec3f(0,0,10),Vec3f(5,0,10)));
	//	objects.push_back(Triangle(Vec3f(0,10,0),Vec3f(0,0,-10),Vec3f(-5,0,-10)));
	//	objects.push_back(Triangle(Vec3f(0,10,0),Vec3f(10,0,0),Vec3f(10,0,5)));
	//	objects.push_back(Triangle(Vec3f(0,10,0),Vec3f(-10,0,0),Vec3f(-10,0,-5)));

	/*	{ // Ladowanie czachy
			FILE *f=fopen("skull1.obj","rb");
			vector<Vec3f> verts,normals,tex;
			for(;!feof(f);) {
				char buf[100];
				fscanf(f,"%s",buf);
				if(strcmp(buf,"v")==0) {
					Vec3f vert;
					fscanf(f,"%f %f %f",&vert.X(),&vert.Z(),&vert.Y());
					vert=(vert-Vec3f(-120,200,-77));
					verts.push_back(vert);
				}
				else if(strcmp(buf,"f")==0) {
					static int q=0; q++;
					if(q==1000) break;
					int a,b,c;
					fscanf(f,"%d %d %d",&a,&b,&c);
					objects.push_back(Triangle(verts[a-1],verts[b-1],verts[c-1]));
				}
				else if(strcmp(buf,"end")==0)
					break;
				else { //if(strcmp(buf,"#")==0)
					char c=0;
					while(c!='\n') fread(&c,1,1,f);
				}
			}
			fclose(f);
		} */
		

	//	AddSoftLight(Vec3f(-10,20.0f,24.5f),Vec3f(800,505,505),Vec3f(40,40,40),1,1,1);
	//	AddSoftLight(Vec3f(-30,-20,0),Vec3f(10,0,1020),Vec3f(40,40,40),1,1,1);
		AddSoftLight(Vec3f(20,-20,40),Vec3f(100,1000,10),Vec3f(40,40,40),1,1,1);
		AddLight(Vec3f(-2,-10,5),Vec3f(1000,1000,1000));
		
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

	template <class Vec,class Group>
	void RayTrace(int packetId,Group &group,const RaySelector<Group::size> &startSelector,Vec *out,int maxRefl) const
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

		tree.TraverseOptimized(PacketIdGenerator::Gen(packetId),group,sel,maxDist,NormalOutput(dst,objId));

		Vec3q nrm[Group::size],colPos[Group::size],reflDir[Group::size];
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

	//		Vec3q refl=Reflect(group.Dir(q),nrm[q]);
	//		out[q].X()=Condition(refl.X()>Const<base,0>::Value(),Const<base,1,12>::Value());
	//		out[q].Y()=Condition(refl.Y()>Const<base,0>::Value(),Const<base,1,12>::Value());
	//		out[q].Z()=Condition(refl.Z()>Const<base,0>::Value(),Const<base,1,12>::Value());
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
			u32 lightObjId[ScalarInfo<base>::Multiplicity*Group::size] __attribute__((aligned(16))); {
				RayGroup<Group::recLevel,1> tGroup(fromLight,&lightPos);
				tree.TraverseOptimized(PacketIdGenerator::Gen(packetId),tGroup,sel,maxDist,ShadowOutput(tDst));
			}

			Vec lightColor=light.color;

			for(int i=0;i<sel.Num();i++) {
				int q=sel.Idx(i);
//				boolv mask=Const<SSEMask,0>::Value();
//				__m128i a=_mm_load_si128((__m128i*)(objId+q*4)),b=_mm_load_si128((__m128i*)(lightObjId+q*4));
//				boolv mask=_mm_castsi128_ps(_mm_cmpeq_epi32(a,b));

				boolv mask=tDst[q]*tDst[q]<lightDistSq[q]*Const<SSEReal,999,1000>::Value();
//				if(ForAll(mask)) // wszystkie punkty zasloniete
//					continue;
				
				base dot=nrm[q]|-fromLight[q]; {
					boolv mask=dot<=Const<base,0>::Value();
					if(ForAll(mask)) continue;
					dot=Condition(!mask,dot);
				}

				out[q]+=Condition(!mask,lightColor*dot*Inv(lightDistSq[q]));
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
			RayTrace(PacketIdGenerator::Gen(packetId),tGroup,sel,reflColor,maxRefl-1);

			for(int i=0;i<sel.Num();i++) {
				int q=sel.Idx(i);
				out[q]=out[q]*Const<base,8,10>::Value()+reflColor[q]*Const<base,2,10>::Value();
			}
		}

		for(int i=0;i<sel.Num();i++) {
			int q=sel[i];
			out[q]=Condition(!tmask[q],out[q]);
		}

		ticks=Ticks()-ticks;

	/*	for(int q=0;q<Group::size;q++) {
			out[q].X()=floatq(sqrt(float(ticks))*0.001f);
			out[q].Y()=floatq(float(0));
			out[q].Z()=floatq(float(0));
		}*/
	}

	vector<Object> objects;

	int nLights;
	Light lights[512];

	SlowKDTree startTree;
	KDTree tree;
};

double ticks=0,lastTicks=0,iters=0;

void GenImage(const Scene &scene,const Camera &cam,Image &out,bool pixDoubling)
{
	unsigned long long tick=Ticks();

	float ratio=float(out.width)/float(out.height);

	enum { QuadLevels=3,
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
			scene.RayTrace(PacketIdGenerator::Gen(),group,allSelector,rgb,0);

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
	float speed=0.7f;
//#endif
	
	bool fullscreen=0,nonInteractive=0;
	int threads=omp_get_num_procs()*8;

	if(argc>2) { resx=atoi(argv[1]); resy=atoi(argv[2]); }
	if(argc>3) { fullscreen=atoi(argv[3])==1; if(atoi(argv[3])>=2) nonInteractive=1; }
	if(argc>4) threads=atoi(argv[4]);

	omp_set_num_threads(threads);
	printf("Threads/cores: %d/%d\n",omp_get_max_threads(),omp_get_num_procs());

	Scene scene;
	if(!scene.tree.Test())
		return 0;

	int full=0,notFull=0;
	for(int n=0;n<scene.tree.objects.size();n++) {
		if(scene.tree.objects[n].fullInNode) full++;
		else notFull++;
	}
	printf("KD Tree params:\nFull KDnode objects: %.2f%%\n\n",double(full)/double(full+notFull));

	Image img(resx,resy);
	Camera cam;
	cam.plane_dist=0.5f;
	//cam.pos=Vec3f(30,20,-75); 
	cam.pos=Vec3f(20,-15,40); 
	//Matrix<Vec4f> rotMat=RotateY(0.7f); cam.right=rotMat*cam.right; cam.front=rotMat*cam.front;
	cam.pos-=cam.front*200.0f;

	if(nonInteractive) {
		for(int n=atoi(argv[3]);n>=0;n--) GenImage(scene,cam,img,0);
		img.SaveToFile("output.tga");
	}
	else {
		SDLOutput out(resx,resy,fullscreen);
		bool showTree=0,pixelDoubling=0;

		while(out.PollEvents()) {
			if(out.TestKey(SDLK_ESCAPE)) break;
			if(out.TestKey(SDLK_p)) img.SaveToFile("output.tga");
			if(out.TestKey(SDLK_t)) showTree^=1;
			if(out.TestKey(SDLK_o)) pixelDoubling^=1;

			if(out.TestKey(SDLK_w)) cam.pos+=cam.front*speed;
			if(out.TestKey(SDLK_s)) cam.pos-=cam.front*speed;

			if(out.TestKey(SDLK_a)) cam.pos-=cam.right*speed;
			if(out.TestKey(SDLK_d)) cam.pos+=cam.right*speed;

			if(out.TestKey(SDLK_r)) cam.pos-=cam.up*speed;
			if(out.TestKey(SDLK_f)) cam.pos+=cam.up*speed;

			if(out.MouseDX()) {
				Matrix<Vec4f> rotMat=RotateY(out.MouseDX()*0.002f);
				cam.right=rotMat*cam.right; cam.front=rotMat*cam.front;
			}
			//if(out.MouseDY()) {
			//	Matrix<Vec4f> rotMat(Quat(AxisAngle(cam.right,-out.MouseDY()*0.002f)));
			//	cam.up=rotMat*cam.up; cam.front=rotMat*cam.front;
			//}
			
			scene.tree.stats.Init();
			int ticks=SDL_GetTicks();
			GenImage(scene,cam,img,pixelDoubling);
			ticks=SDL_GetTicks()-ticks;

			scene.tree.stats.PrintInfo(resx,resy,lastTicks,ticks);

			Vec3f a,b; Convert(scene.tree.pMin,a); Convert(scene.tree.pMax,b);
			if(showTree) scene.startTree.Draw(img,a,b,cam);
			scene.Animate();
			out.Render(img);
		}
	}

	printf("Average cycles/frame: %.2f mln\n",ticks/iters);
	return 0;
}
