#include <iostream>
#include "camera.h"

#include "gl_window.h"
#include "formats/loader.h"
#include "base_scene.h"
#include "font.h"

#include "render.h"
#include "material.h"

#include "bih/tree.h"
#include "tree_box.h"
#include "scene.h"

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

int TreeVisMain(const TriVector&);

int gVals[16]={0,};

using std::cout;
using std::endl;

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

typedef bih::Tree<TriangleVector> StaticTree;
typedef bih::Tree<TreeBoxVector<StaticTree> > FullTree;

class SceneBuilder {
public:
	struct Object {
		Matrix<Vec4f> preTrans;
		StaticTree *tree;
		BBox bBox;
	};
	
	struct Instance {
		Matrix<Vec4f> trans;
		u32 objId;
	};

	// box includes preTrans (it can be more optimized than just tree->BBox()*preTrans)
	void AddObject(StaticTree *tree,const Matrix<Vec4f> &preTrans,const BBox &box) {
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

	typedef TreeBox<StaticTree> Elem;

	TreeBoxVector<StaticTree> ExtractElements() const {
		vector<Elem,AlignedAllocator<Elem> > elements;

		for(int n=0;n<instances.size();n++) {
			const Instance &inst=instances[n];
			const Object &obj=objects[inst.objId];
			elements.push_back(Elem(obj.tree,obj.preTrans*inst.trans,obj.bBox));
		}

		return TreeBoxVector<StaticTree>(elements);
	}
	
	vector<Object> objects;
	vector<Instance> instances;
};

class FrameCounter
{
public:
	FrameCounter() :time(GetTime()),fps(0),frames(0) { }

	void NextFrame() {
		double tTime=GetTime();
		double interval=0.5f;

		frames++;
		if(tTime-time>interval) {
			fps=double(frames)/interval;
			time+=interval;
			frames=0;
		}
	}

	double FPS() const {
		return fps;
	}

private:
	uint frames;
	double time,fps;
};

template <class Dst,class Src>
Dst BitCast(const Src &src) {
	union { Dst d; Src s; } u;
	u.s=src;
	return u.d;
}

template <class Scene>
void SetMaterials(Scene &scene) {
	scene.materials.clear();

	/*
	string names[]={ 	"data/tex316bit.dds",		"data/347.dds",
						"data/1669.dds", 			"data/tex1.png",
						"data/tex2.png",			"data/tex3dxt1.dds",
						"data/ultradxt1.dds",		"", };
	string tnames[]={
		"",
		"scenes/Toasters/Jaw.dds",
		"scenes/Toasters/Head.dds",
		"",
		"scenes/Toasters/Pin.dds",
		"scenes/Toasters/Key.dds",
		"scenes/Toasters/Feet.dds",
		"scenes/Toasters/Stone.dds" };
	for(int n=0;n<sizeof(tnames)/sizeof(string);n++)
		scene.materials.push_back(shading::NewMaterial(tnames[n])); */

	string pre="scenes/sponza/";
	string snames[]={
		"",
		"00_skap.dds",
		"01_S_ba.dds",
		"01_S_kap.dds",
		"01_St_kp.dds",
		"01_STUB.dds",
		"KAMEN.dds",
		"KAMEN-stup.dds",
		"prozor1.dds",
		"reljef.dds",
		"sky.dds",
		"sp_luk.dds",
		"vrata_ko.dds",
		"vrata_kr.dds",
		"x01_st.dds",
		};
	
	vector<Ptr<shading::BaseMaterial>> mats;
	for(int n=0;n<sizeof(snames)/sizeof(string);n++)
		mats.push_back(shading::NewMaterial(n==0?snames[n]:pre+snames[n]));
//	int matid[]={ 0,11,1,0,5,5,2,4,5,14,1,6,0,9,11,6,2,6,8,12,13 };
//	for(int n=0;n<sizeof(matid)/sizeof(int);n++)
//		scene.materials.push_back(mats[matid[n]]);
	scene.materials.push_back(mats[0]);
//	scene.materials.push_back(mats[6]);


}

template <class Scene>
void SetLights(Scene &scene,int max) {
	scene.lights.clear();

	float pos=float(gVals[5])*0.01f;

	if(max>0) scene.lights.push_back(
		/*Toasters*/ //Light(RotateY(pos)*Vec3f(0,400.5f,0),Vec3f(8,8,5)*10000.5f)
		/*sponza*/   Light(Vec3f(0,2,0),Vec3f(8,8,5)*20.0f)
	);
}


int main(int argc, char **argv) {
	printf("Snail v0.11 by nadult\n");
	if(argc>=2&&string("--help")==argv[1]) {
		PrintHelp();
		return 0;
	}
	else printf("type './rtracer --help' to get some help\n");

	CameraConfigs camConfigs;
	try { Loader("scenes/cameras.dat") & camConfigs; } catch(...) { }

	int resx=1024,resy=1024;
#ifndef NDEBUG
	resx/=2; resy/=2;
#endif
	bool fullscreen=0,nonInteractive=0;
	int threads=4,maxLights=1;
	const char *modelFile="barracks.obj";
	Options options;
	bool treeVisMode=0;
	bool flipNormals=1;

	for(int n=1;n<argc;n++) {
			 if(string("-res")==argv[n]&&n<argc-2) { resx=atoi(argv[n+1]); resy=atoi(argv[n+2]); n+=2; }
		else if(string("-threads")==argv[n]&&n<argc-1) { threads=atoi(argv[n+1]); n+=1; }
		else if(string("-fullscreen")==argv[n]) { fullscreen=1; }
		else if(string("-toFile")==argv[n]) { nonInteractive=1; }
		else if(string("-shading")==argv[n]&&n<argc-1) { options.shading=string("gouraud")==argv[n+1]?smGouraud:smFlat; n+=1; }
		else if(string("-treevis")==argv[n]) treeVisMode=1;
		else if(string("+flipNormals")==argv[n]) flipNormals=1;
		else if(string("-flipNormals")==argv[n]) flipNormals=0;
		else modelFile=argv[n];
	}

	printf("Threads/cores: %d/%d\n\n",threads,4);

	printf("Loading...\n");
	BaseScene baseScene; {
		baseScene.LoadWavefrontObj(string("scenes/")+modelFile);
		if(flipNormals) baseScene.FlipNormals();
		baseScene.GenNormals();
		baseScene.Optimize();
	}

	SceneBuilder builder;
	for(int n=0;n<baseScene.objects.size();n++) {
		const BaseScene::Object &obj=baseScene.objects[n];
		builder.AddObject(new StaticTree(obj.ToTriangleVector()),
							obj.GetTrans(),obj.GetBBox());
		builder.AddInstance(n,Identity<>());
	}

	Image img(resx,resy,16);

	double minTime=1.0f/0.0f,maxTime=0.0f;
	
	for(int n=0;n<10;n++) gVals[n]=1;
	gVals[0]=0; gVals[2]=0; gVals[4]=0;

	Scene<StaticTree> staticScene;
	staticScene.geometry.Construct(baseScene.ToTriangleVector());
	staticScene.geometry.PrintInfo();
//	Saver(string("dump/")+modelFile) & staticScene.geometry;
//	Loader(string("dump/")+modelFile) & staticScene.geometry;

	SetMaterials(staticScene);
	SetLights(staticScene,maxLights*0);

	Scene<FullTree> scene;
	SetMaterials(scene);
	SetLights(scene,maxLights*0);

	Camera cam;
	if(!camConfigs.GetConfig(string(modelFile),cam))
		cam.pos=staticScene.geometry.GetBBox().Center();

	if(nonInteractive) {
		double time=GetTime();
		Render(staticScene,cam,img,options,threads);

		time=GetTime()-time;
		minTime=maxTime=time;
		img.SaveToFile("out/output.tga");
	}
	else {
		GLWindow out(resx,resy,fullscreen);
		Font font;

		bool lightsAnim=0,lightsEnabled=0;
		float speed; {
			Vec3p size=staticScene.geometry.GetBBox().Size();
			speed=(size.x+size.y+size.z)*0.005f;
		}

		FrameCounter frmCounter;

		while(out.PollEvents()) {
			frmCounter.NextFrame();

			if(out.KeyUp(Key_esc)) break;
			if(out.KeyDown('K')) img.SaveToFile("out/output.tga");
			if(out.KeyDown('O')) options.reflections^=1;
			if(out.KeyDown('I')) options.rdtscShader^=1;
			if(out.KeyDown('C')) cam.pos=staticScene.geometry.GetBBox().Center();
			if(out.KeyDown('P')) {
				camConfigs.AddConfig(string(modelFile),cam);
				Saver("scenes/cameras.dat") & camConfigs;
				cam.Print();
			}
			if(out.KeyDown('L')) {
				printf("Lights %s\n",lightsEnabled?"disabled":"enabled");
				lightsEnabled^=1;
				SetLights(staticScene,lightsEnabled?maxLights:0);
				SetLights(scene,lightsEnabled?maxLights:0);
			}
			if(out.KeyDown('J')) { printf("Lights animation %s\n",lightsAnim?"disabled":"enabled"); lightsAnim^=1; }

			{
				float tspeed=speed*(out.Key(Key_lshift)?3.0f:1.0f);
				if(out.Key('W')) cam.pos+=cam.front*tspeed;
				if(out.Key('S')) cam.pos-=cam.front*tspeed;
				if(out.Key('A')) cam.pos-=cam.right*tspeed;
				if(out.Key('D')) cam.pos+=cam.right*tspeed;
				if(out.Key('R')) cam.pos+=cam.up*tspeed;
				if(out.Key('F')) cam.pos-=cam.up*tspeed;
			}

		//	if(out.KeyDown('Y')) { printf("splitting %s\n",scene.tree.split?"off":"on"); scene.tree.split^=1; }

			if(out.Key(Key_lctrl)) {
		//		if(out.KeyDown('1')&&scene.lights.size()>=1) scene.lights[0].pos=cam.pos;
		//		if(out.KeyDown('2')&&scene.lights.size()>=2) scene.lights[1].pos=cam.pos;
		//		if(out.KeyDown('3')&&scene.lights.size()>=3) scene.lights[2].pos=cam.pos;
			}

			if(out.KeyDown(Key_f1)) { gVals[0]^=1; printf("Val 1 %s\n",gVals[0]?"on":"off"); }
			if(out.KeyDown(Key_f2)) { gVals[1]^=1; printf("Val 2 %s\n",gVals[1]?"on":"off"); }
			if(out.KeyDown(Key_f3)) { gVals[2]^=1; printf("Val 3 %s\n",gVals[2]?"on":"off"); }
			if(out.KeyDown(Key_f4)) { gVals[3]^=1; printf("Val 4 %s\n",gVals[3]?"on":"off"); }
			if(out.KeyDown(Key_f5)) { gVals[4]^=1; printf("Toggled shading\n"); }
			if(out.KeyDown(Key_f6)) { gVals[5]^=1; printf("Val 5 %s\n",gVals[5]?"old":"new"); }

		//	if(out.KeyDown('U'))
		//		Swap(staticScene.materials[0],staticScene.materials[staticScene.materials.size()==2?1:17]);

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
	
			double buildTime=GetTime();
				scene.geometry.Construct(builder.ExtractElements());
			buildTime=GetTime()-buildTime;

			staticScene.Update();
			scene.Update();
			
			double time=GetTime();
			TreeStats<1> stats;
			if(!gVals[0]) stats=Render(staticScene,cam,img,options,threads);
			else stats=Render(scene,cam,img,options,threads);

			out.RenderImage(img);

			time=GetTime()-time; minTime=Min(minTime,time);
			maxTime=Max(time,maxTime);

			font.BeginDrawing(resx,resy);
			font.SetSize(Vec2f(30,20));
				font.PrintAt(Vec2f(0,0),stats.GenInfo(resx,resy,time*1000.0,buildTime));
				font.PrintAt(Vec2f(0,20),"FPS: ",int(frmCounter.FPS()));
			font.FinishDrawing();

			out.SwapBuffers();


		//	if(lightsAnim) scene.Animate();
		}
	}

	printf("Minimum msec/frame: %.4f (%.2f FPS)\n",minTime*1000.0,1.0f/minTime);
	printf("Maximum msec/frame: %.4f (%.2f FPS)\n",maxTime*1000.0,1.0f/maxTime);

	return 0;
}

