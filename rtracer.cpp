#include <iostream>
#include "camera.h"

#include "gl_window.h"
#include "formats/loader.h"
#include "base_scene.h"

#include "render.h"
#include "shading/material.h"

#include "bvh/tree.h"
//#include "bih/tree.h"

#include "tree_box.h"
#include "scene.h"
#include "mesh.h"
#include "scene_builder.h"
#include "frame_counter.h"

#include "font.h"
#include "frame_counter.h"

int TreeVisMain(const TriVector&);

int gVals[16]={0,};
double gdVals[16]={0,};

using std::cout;
using std::endl;

void PrintHelp() {
	printf("Synopsis:    rtracer model_file [options]\nOptions:\n\t-res x y   - set rendering resolution [512 512]\n\t");
	printf("-fullscreen\n\t-toFile   - renders to file out/output.tga\n\t-threads n   - set threads number to n\n\t");
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

typedef BVH StaticTree;
//typedef bih::Tree<TriangleVector> StaticTree;
//typedef bih::Tree<TreeBoxVector<StaticTree> > FullTree;

template <class Dst,class Src>
Dst BitCast(const Src &src) {
	union { Dst d; Src s; } u;
	u.s=src;
	return u.d;
}

template <class Scene>
void SetMaterials(Scene &scene,const BaseScene &base,string texPath) {
	scene.materials.clear();
//	for(int n=0;n<128;n++)
//		scene.materials.push_back(shading::NewMaterial(""));
//	scene.materials[1]=shading::NewMaterial("data/ultradxt1.dds");
//	return;

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
		scene.materials.push_back(shading::NewMaterial(tnames[n]));
	scene.materials[1]=shading::NewMaterial("data/ultradxt1.dds"); */
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
//	scene.materials.push_back(mats[6]); */

	scene.materials.resize(base.matNames.size());
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
//	scene.materials.back()=typename Scene::PMaterial(shading::NewMaterial("scenes/doom3/imp/texture.tga"));

	if(scene.materials.size()>126) {
		scene.materials[126]->flags|=shading::Material::fReflection;
		scene.materials[ 70]->flags|=/*shading::Material::fRefraction|*/shading::Material::fReflection;
	}
}

vector<Light> GenLights(float scale = 1.0f, float power = 1000.0f) {
	vector<Light> out;

	float pos=float(gVals[5])*0.01f;

	out.push_back(
		/*Toasters*/ //Light(RotateY(pos)*Vec3f(0,400.5f,0),Vec3f(8,8,5),10000.f)
		/*sponza*/  // Light(Vec3f(0,2,0),Vec3f(8,8,5),20.0f)
		/*admin*/   Light(Vec3f(-78.0f,110.0f,-531.0f) * scale, Vec3f(1,1,0.7), power)
	);
//	out.push_back(Light(Vec3f(-600.0f,144.0f,-341.0f),Vec3f(0.3,0.6,1.0),800.0f));
//	out.push_back(Light(Vec3f(407.0f,209.64f,1634.0f),Vec3f(1,1,1),1000.0f));

	return out;
}

namespace game { int main(int argc,char **argv); }

static int tmain(int argc, char **argv) {
	printf("Snail v0.11 by nadult\n");
	if(argc>=2&&string("--help")==argv[1]) {
		PrintHelp();
		return 0;
	}
	else printf("type './rtracer --help' to get some help\n");

	CameraConfigs camConfigs;
	try { Loader("scenes/cameras.dat") & camConfigs; } catch(...) { }

//	Mesh mesh;
//	MeshAnim meshAnim;
//	mesh.Load("scenes/doom3/imp/imp.md5mesh");
//	meshAnim.Load("scenes/doom3/imp/walk1.md5anim");

//	StaticTree meshTree;

	int resx=1280, resy=720;
#ifndef NDEBUG
	resx/=2; resy/=2;
#endif
	bool fullscreen=0;
	int threads=4;
	const char *modelFile="pompei.obj";//"doom3/admin.proc";

	Options options;
	bool flipNormals = 1;
	bool rebuild = 0;
	string texPath="/mnt/data/data/doom3/";

	for(int n=1;n<argc;n++) {
			 if(string("-res")==argv[n]&&n<argc-2) { resx=atoi(argv[n+1]); resy=atoi(argv[n+2]); n+=2; }
		else if(string("-rebuild") == argv[n]) { rebuild = 1; }
		else if(string("-threads")==argv[n]&&n<argc-1) { threads=atoi(argv[n+1]); n+=1; }
		else if(string("-fullscreen")==argv[n]) { fullscreen=1; }
		else if(string("+flipNormals")==argv[n]) flipNormals=1;
		else if(string("-flipNormals")==argv[n]) flipNormals=0;
		else if(string("-texPath")==argv[n]) { texPath=argv[n+1]; n++; }
		else {
			if(argv[n][0]=='-') printf("Unknown option: %s\n",argv[n]);
			else modelFile=argv[n];
		}
	}

//	if(gameMode) return game::main(argc,argv);

	printf("Threads/cores: %d/%d\n\n",threads,4);

	printf("Loading...\n");
	double buildTime = GetTime();
	BaseScene baseScene; {
		string fileName = string("scenes/")+modelFile;
		if(fileName.find(".proc") != string::npos)
			baseScene.LoadDoom3Proc(fileName);
		else if(fileName.find(".obj") != string::npos)
			baseScene.LoadWavefrontObj(fileName);

		int tris = 0;
		for(int n = 0; n < baseScene.objects.size(); n++)
			tris += baseScene.objects[n].tris.size();
		printf("Tris: %d\n",tris);

		if(flipNormals) baseScene.FlipNormals();
		for(int n = 0; n < baseScene.objects.size(); n++)
			baseScene.objects[n].Repair();
		baseScene.GenNormals();
//		baseScene.Optimize();
	}
	
	Scene<StaticTree> staticScene;
	if(!rebuild) {
		try { Loader(string("dump/") + modelFile) & staticScene.geometry; }
		catch(...) { rebuild = 1; }
	}
	if(rebuild) {
		staticScene.geometry.Construct(baseScene.ToTriVector());
	//	staticScene.geometry.Construct(baseScene.ToTriangleVector());
		Saver(string("dump/") + modelFile) & staticScene.geometry;
	}
	staticScene.geometry.PrintInfo();

	float sceneScale; {
		Vec3f bMin(1.0f / 0.0f, 1.0f / 0.0f, 1.0f / 0.0f);
		Vec3f bMax = -bMin;
		for(int n = 0; n < baseScene.objects.size(); n++) {
			auto box = baseScene.objects[n].GetBBox();
			bMin = VMin(bMin, box.min);
			bMax = VMax(bMax, box.max);
		}
		Vec3f size = bMax - bMin;
		sceneScale = Length(size);
	}

	SceneBuilder<StaticTree> builder; /*{
		vector<BaseScene::Object> objects;

		for(int n=0;n<baseScene.objects.size();n++) {
			const BaseScene::Object &obj=baseScene.objects[n];
			bool contained=0;
			for(int k=0;k<objects.size();k++) {
				BaseScene::Object &tObj=objects[k];
				if(obj.GetBBox().Contains(tObj.GetBBox(),1.5f)||tObj.GetBBox().Contains(obj.GetBBox(),1.5f)) {
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
	}*/ {
		builder.AddObject(&staticScene.geometry,Identity<>(),staticScene.geometry.GetBBox());
		builder.AddInstance(0,Identity<>());
	}

	GLWindow window(resx, resy, fullscreen);
	Font font;

	gfxlib::Texture image(resx, resy, gfxlib::TI_A8B8G8R8);

	double minTime=1.0f/0.0f,maxTime=0.0f;
	
	for(int n=0;n<10;n++) gVals[n]=1;
	gVals[0] = 0; gVals[2]=0; gVals[4]=0; gVals[3]=0;
	
	gdVals[0] = 0.1f * sceneScale;
	gdVals[1] = 0.2f * sceneScale;

	SetMaterials(staticScene, baseScene, texPath);

//	Scene<FullTree> scene;
//	SetMaterials(scene, baseScene, texPath);
//	mesh.SetMaterial(scene.materials.size()-1);

	vector<Light> lights;// = GenLights();
			
	Camera cam;
	if(!camConfigs.GetConfig(string(modelFile),cam))
		cam.pos=staticScene.geometry.GetBBox().Center();

	buildTime=GetTime()-buildTime;
	std::cout << "Build time: " << buildTime << '\n';

	bool lightsEnabled=1;
	bool staticEnabled=0;
	float speed; {
	//	scene.geometry.Construct(builder.ExtractElements());
		Vec3p size = staticScene.geometry.GetBBox().Size();
		speed=(size.x + size.y + size.z) * 0.0025f;
	}

	FrameCounter frmCounter;

	while(window.PollEvents()) {
		frmCounter.NextFrame();

		if(window.KeyUp(Key_esc)) break;
		if(window.KeyDown('K')) Saver("out/output.dds") & image;
		if(window.KeyDown('O')) options.reflections^=1;
		if(window.KeyDown('I')) options.rdtscShader^=1;
		if(window.KeyDown('C')) {
		//	if(staticEnabled)
				cam.pos=staticScene.geometry.GetBBox().Center();
		//	else cam.pos=scene.geometry.GetBBox().Center();
		}
		if(window.KeyDown('P')) {
			camConfigs.AddConfig(string(modelFile),cam);
			Saver("scenes/cameras.dat") & camConfigs;
			cam.Print();
		}
		if(window.KeyDown('L')) {
			printf("Lights %s\n",lightsEnabled?"disabled":"enabled");
			lightsEnabled^=1;
		}
		if(window.KeyDown('J')) {
			Vec3f colors[4]={Vec3f(1,1,1),Vec3f(0.2,0.5,1),Vec3f(0.5,1,0.2),Vec3f(0.7,1.0,0.0)};

			lights.push_back(Light(cam.pos, colors[rand()&3], 300.0f * 0.001f * sceneScale));
		}

		{
			float tspeed=speed*(window.Key(Key_lshift)?5.0f:1.0f);
			if(window.Key('W')) cam.pos+=cam.front*tspeed;
			if(window.Key('S')) cam.pos-=cam.front*tspeed;
			if(window.Key('A')) cam.pos-=cam.right*tspeed;
			if(window.Key('D')) cam.pos+=cam.right*tspeed;
			if(window.Key('R')) cam.pos+=cam.up*tspeed;
			if(window.Key('F')) cam.pos-=cam.up*tspeed;
		}

		if(window.Key('G')) { gdVals[0]+=0.01 * sceneScale; printf("16 max dist: %f\n", gdVals[0] / sceneScale); }
		if(window.Key('V')) { gdVals[0]-=0.01 * sceneScale; printf("16 max dist: %f\n", gdVals[0] / sceneScale); }
		if(window.Key('H')) { gdVals[1]+=0.01 * sceneScale; printf("4 max dist: %f\n", gdVals[1] / sceneScale); }
		if(window.Key('B')) { gdVals[1]-=0.01 * sceneScale; printf("4 max dist: %f\n", gdVals[1] / sceneScale); }

		for(int n = 1; n <= 8; n++) if(window.Key('0' + n))
				{ threads = n; printf("Threads: %d\n", threads); }

		if(window.KeyDown(Key_f1)) { gVals[0]^=1; printf("Traversing from 8x8: %s\n", gVals[0]?"on" : "off"); }
		if(window.KeyDown(Key_f2)) { gVals[1]^=1; printf("Val 2 %s\n",gVals[1]?"on":"off"); }
		if(window.KeyDown(Key_f3)) { gVals[2]^=1; printf("Val 3 %s\n",gVals[2]?"on":"off"); }
		if(window.KeyDown(Key_f4)) { gVals[3]^=1; printf("Val 4 %s\n",gVals[3]?"on":"off"); }
		if(window.KeyDown(Key_f5)) { gVals[4]^=1; printf("Toggled shading\n"); }
		if(window.KeyDown(Key_f6)) { gVals[5]^=1; printf("Val 5 %s\n",gVals[5]?"on":"off"); }
		if(window.KeyDown(Key_f7)) { gVals[6]^=1; printf("Val 6 %s\n",gVals[6]?"on":"off"); }
		if(window.KeyDown(Key_f8)) { gVals[7]^=1; printf("Val 7 %s\n",gVals[7]?"on":"off"); }

		{
			int dx=window.Key(Key_space)?window.MouseMove().x:0,dy=0;
			if(window.Key('N')) dx-=20;
			if(window.Key('M')) dx+=20;
			if(window.Key('V')) dy-=20;
			if(window.Key('B')) dy+=20;
			if(dx) {
				Matrix<Vec4f> rotMat=RotateY(-dx*0.003f);
				cam.right=rotMat*cam.right; cam.front=rotMat*cam.front;
			}
		//	if(dy) {
		//		Matrix<Vec4f> rotMat(Quat(AxisAngle(cam.right,-dy*0.002f)));
		//		cam.up=rotMat*cam.up; cam.front=rotMat*cam.front;
		//	}
		}

		static float animPos = 0;
		if(window.Key(Key_space)) animPos+=0.025f;

		double buildTime=GetTime(); /*{
			SceneBuilder<StaticTree> temp=builder;
		//	for(int n=0;n<temp.instances.size();n++) {
		//		SceneBuilder<StaticTree>::Instance &inst=temp.instances[n];
		//		inst.trans.w=inst.trans.w+Vec4f(0.0f,sin(animPos+n*n)*5.0f*speed,0.0f,0.0f);
		//	}
			mesh.Animate(meshAnim,animPos);
			meshTree.Construct(mesh.triVec,1);
	//		staticScene.geometry = meshTree;

			temp.AddObject(&meshTree,Identity<>(),meshTree.GetBBox());
			for(int x=-1;x<2;x++) for(int z=-1;z<2;z++)
				temp.AddInstance(temp.objects.size()-1,
						RotateY(3.1415f)*Translate(Vec3f(132 + x * 50, 0, z * 50 - 346)));
			scene.geometry.Construct(temp.ExtractElements(),1);
			BBox box=meshTree.GetBBox();

					} */ buildTime=GetTime()-buildTime;

		vector<Light> tLights = lightsEnabled?lights:vector<Light>();
			for(int n=0;n<tLights.size();n++)
				tLights[n].pos += Vec3f(sin(animPos+n*n),cos(animPos+n*n),
					sin(animPos-n*n)*cos(animPos+n*n))*speed;
		staticScene.lights /*= scene.lights*/ = tLights;
		staticScene.Update();
	//	scene.Update();
		
		double time=GetTime();
		TreeStats<1> stats;
	//	if(staticEnabled)
			stats = Render(staticScene, cam, image, options, threads);
	//	else stats=Render(scene,cam,image,options,threads);

		time=GetTime()-time; minTime=Min(minTime,time);
		maxTime=Max(time,maxTime);

		window.RenderImage(image);

		double fps = double(unsigned(frmCounter.FPS() * 100)) * 0.01;
		double mrays = double(unsigned(frmCounter.FPS() * stats.GetRays() * 0.0001)) * 0.01;

		font.BeginDrawing(resx,resy);
		font.SetSize(Vec2f(30, 20));
			font.PrintAt(Vec2f(5,  5), stats.GenInfo(resx, resy, time * 1000.0,buildTime * 1000.0));
			font.PrintAt(Vec2f(5, 25), "FPS: ", fps, " MRays/sec:", mrays);
			if(lightsEnabled && lights.size())
				font.PrintAt(Vec2f(5, 45), "Lights: ",lightsEnabled?lights.size() : 0);
			font.PrintAt(Vec2f(5, 85), "prim:", gVals[1], ' ', gVals[2], ' ', gVals[3],
					" sh:", gVals[4], " refl:", gVals[5], ' ', gVals[6], " smart:", gVals[7], ' ',
					gdVals[0] / sceneScale, ' ', gdVals[1] / sceneScale);
		font.FinishDrawing();
		window.SwapBuffers();
	}

	printf("Minimum msec/frame: %.4f (%.2f FPS)\n",minTime*1000.0,1.0f/minTime);
	printf("Maximum msec/frame: %.4f (%.2f FPS)\n",maxTime*1000.0,1.0f/maxTime);

	return 0;
}

int main(int argc,char **argv) {
	try {
		return tmain(argc,argv);
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
