#include <iostream>
#include "camera.h"

#include "gl_window.h"
#include "formats/loader.h"
#include "base_scene.h"
#include "font.h"

#include "render.h"
#include "shading/material.h"

#include "bih/tree.h"
#include "tree_box.h"
#include "scene.h"
#include "mesh.h"

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
			Matrix<Vec4f> mat=obj.preTrans*inst.trans;
			BBox box=obj.bBox*inst.trans;
			elements.push_back(Elem(obj.tree,mat,box));
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
void SetMaterials(Scene &scene,const BaseScene &base,string texPath) {
	scene.materials.clear();

/*	string names[]={ 	"data/tex316bit.dds",		"data/347.dds",
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

	/*
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
	
	vector<Ptr<shading::Material>> mats;
	for(int n=0;n<sizeof(snames)/sizeof(string);n++)
		mats.push_back(shading::NewMaterial(n==0?snames[n]:pre+snames[n]));
//	int matid[]={ 0,11,1,0,5,5,2,4,5,14,1,6,0,9,11,6,2,6,8,12,13 };
//	for(int n=0;n<sizeof(matid)/sizeof(int);n++)
//		scene.materials.push_back(mats[matid[n]]);
//	scene.materials.push_back(mats[0]);
//	scene.materials.push_back(mats[6]);
*/
	scene.materials.resize(base.matNames.size()+1);
	for(std::map<string,int>::const_iterator it=base.matNames.begin();it!=base.matNames.end();++it) {
		string name=it->first==""?"":texPath+it->first;

		try {
		 	scene.materials[it->second]=typename Scene::PMaterial(shading::NewMaterial(name));
		}
		catch(const Exception &ex) {
			std::cout << ex.what() << '\n';
			scene.materials[it->second]=scene.materials.front();
		}
	}
	scene.materials.back()=typename Scene::PMaterial(shading::NewMaterial("scenes/doom3/imp/texture.tga"));

	if(scene.materials.size()>126) {
		scene.materials[126]->flags|=shading::Material::fReflection;
		scene.materials[ 70]->flags|=shading::Material::fRefraction|shading::Material::fReflection;
	}
}

vector<Light> GenLights() {
	vector<Light> out;

	float pos=float(gVals[5])*0.01f;

	out.push_back(
		/*Toasters*/ //Light(RotateY(pos)*Vec3f(0,400.5f,0),Vec3f(8,8,5),10000.f)
		/*sponza*/  // Light(Vec3f(0,2,0),Vec3f(8,8,5),20.0f)
		/*admin*/   Light(Vec3f(-78.0f,110.0f,-531.0f),Vec3f(1,1,0.7),1000.0f)
	);
//	out.push_back(Light(Vec3f(-600.0f,144.0f,-341.0f),Vec3f(0.3,0.6,1.0),800.0f));
//	out.push_back(Light(Vec3f(407.0f,209.64f,1634.0f),Vec3f(1,1,1),1000.0f));

	return out;
}


int safe_main(int argc, char **argv) {
	printf("Snail v0.11 by nadult\n");
	if(argc>=2&&string("--help")==argv[1]) {
		PrintHelp();
		return 0;
	}
	else printf("type './rtracer --help' to get some help\n");

	CameraConfigs camConfigs;
	try { Loader("scenes/cameras.dat") & camConfigs; } catch(...) { }

	Mesh mesh;
	MeshAnim meshAnim;
	mesh.Load("scenes/doom3/imp/imp.md5mesh");
	meshAnim.Load("scenes/doom3/imp/walk1.md5anim");

	StaticTree meshTree;

	int resx=800,resy=600;
#ifndef NDEBUG
	resx/=2; resy/=2;
#endif
	bool fullscreen=0,nonInteractive=0;
	int threads=4;
	const char *modelFile="doom3/admin.proc";
	Options options;
	bool treeVisMode=0;
	bool flipNormals=1;
	string texPath="/mnt/Data/data/doom3/";

	for(int n=1;n<argc;n++) {
			 if(string("-res")==argv[n]&&n<argc-2) { resx=atoi(argv[n+1]); resy=atoi(argv[n+2]); n+=2; }
		else if(string("-threads")==argv[n]&&n<argc-1) { threads=atoi(argv[n+1]); n+=1; }
		else if(string("-fullscreen")==argv[n]) { fullscreen=1; }
		else if(string("-toFile")==argv[n]) { nonInteractive=1; }
		else if(string("-treevis")==argv[n]) treeVisMode=1;
		else if(string("+flipNormals")==argv[n]) flipNormals=1;
		else if(string("-flipNormals")==argv[n]) flipNormals=0;
		else if(string("-texPath")==argv[n]) { texPath=argv[n+1]; n++; }
		else modelFile=argv[n];
	}

	printf("Threads/cores: %d/%d\n\n",threads,4);

	printf("Loading...\n");
	double buildTime=GetTime();
	BaseScene baseScene; {
		string fileName=string("scenes/")+modelFile;
		if(fileName.find(".proc")!=string::npos)
			baseScene.LoadDoom3Proc(fileName);
		else if(fileName.find(".obj")!=string::npos)
			baseScene.LoadWavefrontObj(fileName);

		if(flipNormals) baseScene.FlipNormals();
		for(int n=0;n<baseScene.objects.size();n++)
			baseScene.objects[n].Repair();
	//	baseScene.GenNormals();
	//	baseScene.Optimize();
	}

	SceneBuilder builder; {
		vector<BaseScene::Object> objects;

		for(int n=0;n<baseScene.objects.size();n++) {
			const BaseScene::Object &obj=baseScene.objects[n];
			bool contained=0;
			for(int k=0;k<objects.size();k++) {
				BaseScene::Object &tObj=objects[k];
				if(1||obj.GetBBox().Contains(tObj.GetBBox(),1.5f)||tObj.GetBBox().Contains(obj.GetBBox(),1.5f)) {
					contained=1;
					tObj.Join(obj);
					break;
				}
			}
			if(!contained) objects.push_back(baseScene.objects[n]);

		}

		int count=0;	
		for(int o=0;o<objects.size();o++) {
			BaseScene::Object &obj=objects[o];
		//	obj.Optimize();
			TriangleVector vec=obj.ToTriangleVector();
			if(vec.size()) {
				builder.AddObject(new StaticTree(vec),obj.GetTrans(),obj.GetBBox());
				builder.AddInstance(count++,Identity<>());
			}
		}
	}

	Image img(resx,resy,16);

	double minTime=1.0f/0.0f,maxTime=0.0f;
	
	for(int n=0;n<10;n++) gVals[n]=1;
	gVals[2]=0; gVals[4]=0; gVals[3]=0;

	Scene<StaticTree> staticScene;
	staticScene.geometry.Construct(baseScene.ToTriangleVector());
	staticScene.geometry.PrintInfo();
	Saver(string("dump/")+modelFile) & staticScene.geometry;
//	Loader(string("dump/")+modelFile) & staticScene.geometry;

	SetMaterials(staticScene,baseScene,texPath);

	Scene<FullTree> scene;
	SetMaterials(scene,baseScene,texPath);
	mesh.SetMaterial(scene.materials.size()-1);

	vector<Light> lights=GenLights();
			
	Camera cam;
	if(!camConfigs.GetConfig(string(modelFile),cam))
		cam.pos=scene.geometry.GetBBox().Center();

	buildTime=GetTime()-buildTime;
	std::cout << "Build time: " << buildTime << '\n';

	if(nonInteractive) {
		double time=GetTime();
	//	ThrowException("Rendering to file disabled");

		staticScene.Update();
		Render(staticScene,cam,img,options,threads);

		time=GetTime()-time;
		minTime=maxTime=time;
		img.SaveToFile("out/output.tga");
	}
	else {
		GLWindow out(resx,resy,fullscreen);
		Font font;

		bool lightsEnabled=1;
		bool staticEnabled=0;
		float speed; {
			scene.geometry.Construct(builder.ExtractElements());
			Vec3p size=staticScene.geometry.GetBBox().Size();
			speed=(size.x+size.y+size.z)*0.0025f;
		}

		FrameCounter frmCounter;

		while(out.PollEvents()) {
			frmCounter.NextFrame();

			if(out.KeyUp(Key_esc)) break;
			if(out.KeyDown('K')) img.SaveToFile("out/output.tga");
			if(out.KeyDown('O')) options.reflections^=1;
			if(out.KeyDown('I')) options.rdtscShader^=1;
			if(out.KeyDown('C')) cam.pos=scene.geometry.GetBBox().Center();
			if(out.KeyDown('P')) {
				camConfigs.AddConfig(string(modelFile),cam);
				Saver("scenes/cameras.dat") & camConfigs;
				cam.Print();
			}
			if(out.KeyDown('L')) {
				printf("Lights %s\n",lightsEnabled?"disabled":"enabled");
				lightsEnabled^=1;
			}
			if(out.KeyDown('J')) {
				Vec3f colors[4]={Vec3f(1,1,1),Vec3f(0.2,0.5,1),Vec3f(0.5,1,0.2),Vec3f(0.7,1.0,0.0)};

				lights.push_back(Light(cam.pos,colors[rand()&3],800.0f));
			}

			{
				float tspeed=speed*(out.Key(Key_lshift)?5.0f:1.0f);
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

			if(out.KeyDown(Key_f1)) staticEnabled^=1;
			if(out.KeyDown(Key_f2)) { gVals[1]^=1; printf("Val 2 %s\n",gVals[1]?"on":"off"); }
			if(out.KeyDown(Key_f3)) { gVals[2]^=1; printf("Val 3 %s\n",gVals[2]?"on":"off"); }
			if(out.KeyDown(Key_f4)) { gVals[3]^=1; printf("Val 4 %s\n",gVals[3]?"on":"off"); }
			if(out.KeyDown(Key_f5)) { gVals[4]^=1; printf("Toggled shading\n"); }
			if(out.KeyDown(Key_f6)) { gVals[5]^=1; printf("Val 5 %s\n",gVals[5]?"on":"off"); }

		/*	if(out.KeyDown('U')) {
				static int n=1;
				Swap(staticScene.materials[0],staticScene.materials[n]);
				Swap(scene.materials[0],scene.materials[n]);
				printf("%d toggled\n",n);
				n++;
			} */

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
	
			double buildTime=GetTime(); {
				static float pos=0.0f; if(out.Key(Key_space)) pos+=0.025f;
				SceneBuilder temp=builder;
			//	for(int n=0;n<temp.instances.size();n++) {
			//		SceneBuilder::Instance &inst=temp.instances[n];
			//		inst.trans.w=inst.trans.w+Vec4f(0.0f,sin(pos+n*n)*5.0f*speed,0.0f,0.0f);
			//	}
				mesh.Animate(meshAnim,pos);
				meshTree.Construct(mesh.triVec,1);
			//	staticScene.geometry=meshTree;

				temp.AddObject(&meshTree,Identity<>(),meshTree.GetBBox());
				for(int x=-1;x<2;x++) for(int z=-1;z<2;z++)
					temp.AddInstance(temp.objects.size()-1,
							RotateY(3.1415f)*Translate(Vec3f(132+x*50,0,z*50-346)));
				scene.geometry.Construct(temp.ExtractElements(),1);
				BBox box=meshTree.GetBBox();

				vector<Light> tLights=lightsEnabled?lights:vector<Light>();
				for(int n=0;n<tLights.size();n++) {
					tLights[n].pos += Vec3f(sin(pos+n*n),cos(pos+n*n),sin(pos-n*n)*cos(pos+n*n))*speed;
				}
				staticScene.lights=scene.lights=tLights;
			buildTime=GetTime()-buildTime; }

			staticScene.Update();
			scene.Update();
			
			double time=GetTime();
			TreeStats<1> stats;
			if(staticEnabled) stats=Render(staticScene,cam,img,options,threads);
			else stats=Render(scene,cam,img,options,threads);

			out.RenderImage(img);

			time=GetTime()-time; minTime=Min(minTime,time);
			maxTime=Max(time,maxTime);

			font.BeginDrawing(resx,resy);
			font.SetSize(Vec2f(30,20));
				font.PrintAt(Vec2f(0,0),stats.GenInfo(resx,resy,time*1000.0,buildTime*1000.0));
				font.PrintAt(Vec2f(0,20),"FPS (",staticEnabled?"static":"dynamic","): ",int(frmCounter.FPS()));
				font.PrintAt(Vec2f(0,40),"Lights: ",lightsEnabled?int(lights.size()):0);

			font.FinishDrawing();

			out.SwapBuffers();
		}
	}

	printf("Minimum msec/frame: %.4f (%.2f FPS)\n",minTime*1000.0,1.0f/minTime);
	printf("Maximum msec/frame: %.4f (%.2f FPS)\n",maxTime*1000.0,1.0f/maxTime);

	return 0;
}

int main(int argc,char **argv) {
	try {
		return safe_main(argc,argv);
	}
	catch(const std::exception &ex) {
		std::cout << ex.what() << '\n';
		return 1;
	}
	catch(...) {
		std::cout << "Unknown exception thrown.\n";
		throw;
	}
}
