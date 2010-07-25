#include <iostream>
#include "camera.h"

#include "gl_window.h"
#include "formats/loader.h"
#include "base_scene.h"

#include "render.h"
#include "shading/material.h"

#include "bvh/tree.h"
#include "dbvh/tree.h"

#include "scene.h"
#include "frame_counter.h"

#include "font.h"
#include "frame_counter.h"

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
	printf("F1 - toggle shadow caching\n\t");
	printf("esc - exit\n\n");
}

typedef BVH StaticTree;

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

static void MoveCamera(Camera &cam, GLWindow &window, float speed) {
	if(window.MouseKey(0)) {
		float dx = window.MouseMove().x;
		float dy = window.MouseMove().y;

		if(dx) cam.Rotate(-dx * 0.001);
		if(dy) cam.RotateY(dy * 0.001);
		window.GrabMouse(1);
	}
	else window.GrabMouse(0);

	Vec3f move(0, 0, 0);
	Vec3f right, up, front;
	cam.GetRotation(right, up, front);

	if(window.Key('W')) move += front;
	if(window.Key('S')) move -= front;
	if(window.Key('A')) move -= right;
	if(window.Key('D')) move += right;
	if(window.Key('R')) move += up;
	if(window.Key('F')) move -= up;

	cam.Move(move * speed * (window.Key(Key_lshift)? 5.0f : 1.0f));
}

static const DBVH MakeDBVH(BVH *bvh) {
	srand(0);
	static float anim = 0; anim += 0.02f;

	float scale = Length(bvh->GetBBox().Size()) * 0.025;

	vector<ObjectInstance> instances;
	for(int n = 0; n < 1000; n++) {
		ObjectInstance inst;
		inst.tree = bvh;
		inst.translation = (Vec3f(rand() % 1000, rand() % 1000, rand() % 1000) - Vec3f(500, 500, 500))
			* scale;
		Matrix<Vec4f> rot = Rotate(
				(rand() % 1000) * 0.002f * constant::pi + anim,
				(rand() % 1000) * 0.002f * constant::pi + anim * 0.2f,
				(rand() % 1000) * 0.002f * constant::pi + anim * 0.1f);
		inst.rotation[0] = Vec3f(rot.x);
		inst.rotation[1] = Vec3f(rot.y);
		inst.rotation[2] = Vec3f(rot.z);
		inst.ComputeBBox();
		instances.push_back(inst);
	}
	ObjectInstance zero;
	zero.tree = bvh;
	zero.translation = Vec3f(0, 0, 0);
	zero.rotation[0] = Vec3f(1, 0, 0);
	zero.rotation[1] = Vec3f(0, 1, 0);
	zero.rotation[2] = Vec3f(0, 0, 1);
	zero.ComputeBBox();
	instances.push_back(zero);

	return DBVH(instances);
}

static int tmain(int argc, char **argv) {
	printf("Snail v0.20 by nadult\n");
	if(argc>=2&&string("--help")==argv[1]) {
		PrintHelp();
		return 0;
	}
	else printf("type './rtracer --help' to get some help\n");

	CameraConfigs camConfigs;
	try { Loader("scenes/cameras.dat") & camConfigs; } catch(...) { }

	int resx = 1024, resy = 1024;
	bool fullscreen = 0;
	int threads = 2;
	string sceneName = "pompei.obj";//"doom3/admin.proc";

	Options options;
	bool flipNormals = 1;
	bool rebuild = 0, slowBuild = 0;
	string texPath = "";

	for(int n=1;n<argc;n++) {
			 if(string("-res")==argv[n]&&n<argc-2) { resx=atoi(argv[n+1]); resy=atoi(argv[n+2]); n+=2; }
		else if(string("-rebuild") == argv[n]) { rebuild = 1; }
		else if(string("-slowBuild") == argv[n]) { rebuild = 1; slowBuild = 1; }
		else if(string("-threads")==argv[n]&&n<argc-1) { threads=atoi(argv[n+1]); n+=1; }
		else if(string("-fullscreen")==argv[n]) { fullscreen=1; }
		else if(string("+flipNormals")==argv[n]) flipNormals=1;
		else if(string("-flipNormals")==argv[n]) flipNormals=0;
		else if(string("-texPath")==argv[n]) { texPath=argv[n+1]; n++; }
		else {
			if(argv[n][0] == '-') printf("Unknown option: %s\n",argv[n]);
			else sceneName = argv[n];
		}
	}

	if(texPath == "") {
		texPath = "scenes/" + sceneName;
		int pos = texPath.rfind('/');
		if(pos == string::npos) texPath = "";
		else texPath.resize(pos + 1);
	}

	Scene<StaticTree> scene;
	if(!rebuild) {
		try { Loader(string("dump/") + sceneName) & scene.geometry; }
		catch(...) { rebuild = 1; }
	}
	if(rebuild) {
		printf("Loading...\n");
		double loadingTime = GetTime();
		BaseScene baseScene; {
			string fileName = string("scenes/") + sceneName;
			if(fileName.find(".proc") != string::npos)
				baseScene.LoadDoom3Proc(fileName);
			else if(fileName.find(".obj") != string::npos)
				baseScene.LoadWavefrontObj(fileName);
			else ThrowException("Unrecognized format: ", fileName);

			int tris = 0;
			for(int n = 0; n < baseScene.objects.size(); n++)
				tris += baseScene.objects[n].tris.size();
			printf("Tris: %d\n",tris);

			if(flipNormals) baseScene.FlipNormals();
		//	for(int n = 0; n < baseScene.objects.size(); n++)
		//		baseScene.objects[n].Repair();
			baseScene.GenNormals();
		//	baseScene.Optimize();
		}
		loadingTime = GetTime() - loadingTime;
		
		double buildTime = GetTime();
		scene.geometry.Construct(baseScene.ToCompactTris(), baseScene.matNames, !slowBuild);
	//	scene.geometry.Construct(baseScene.ToTriangleVector());
		Saver(string("dump/") + sceneName) & scene.geometry;
		buildTime = GetTime() - buildTime;
		std::cout << "Loading time: " << loadingTime << "  Build time: " << buildTime << '\n';
	}
	scene.geometry.PrintInfo();
	scene.geometry.UpdateCache();

	if(sceneName.substr(sceneName.size() - 4) == ".obj")
		scene.matDict = shading::LoadMaterials("scenes/" + sceneName.substr(0, sceneName.size() - 3) + "mtl", texPath);
	scene.UpdateMaterials();
	scene.geometry.UpdateMaterialIds(scene.GetMatIdMap());

	float sceneScale; {
		Vec3f size = scene.geometry.GetBBox().Size();
		sceneScale = Length(size);
	}

	GLWindow window(resx, resy, fullscreen);
	Font font;

	gfxlib::Texture image(resx, resy, gfxlib::TI_A8R8G8B8);
	vector<Light> lights;// = GenLights();
			
	Camera cam;
	if(!camConfigs.GetConfig(string(sceneName),cam))
		cam.SetPos(scene.geometry.GetBBox().Center());

	bool lightsEnabled = 1;
	bool staticEnabled = 0;
	float speed; {
	//	scene.geometry.Construct(builder.ExtractElements());
		Vec3p size = scene.geometry.GetBBox().Size();
		speed = (size.x + size.y + size.z) * 0.0025f;
	}

	FrameCounter frmCounter;

	while(window.PollEvents()) {
		frmCounter.NextFrame();

		if(window.KeyUp(Key_esc)) break;
		if(window.KeyDown('K')) Saver("out/output.dds") & image;
		if(window.KeyDown('O')) options.reflections ^= 1;
		if(window.KeyDown('I')) options.rdtscShader ^= 1;
		if(window.KeyDown('C')) {
		//	if(staticEnabled)
				cam.SetPos(scene.geometry.GetBBox().Center());
		//	else cam.pos=scene.geometry.GetBBox().Center();
		}
		if(window.KeyDown('P')) {
			camConfigs.AddConfig(string(sceneName),cam);
			Saver("scenes/cameras.dat") & camConfigs;
			cam.Print();
		}
		if(window.KeyDown('L')) {
			printf("Lights %s\n",lightsEnabled?"disabled":"enabled");
			lightsEnabled^=1;
		}
		if(window.KeyDown('J')) {
			Vec3f colors[4]={Vec3f(1,1,1),Vec3f(0.2,0.5,1),Vec3f(0.5,1,0.2),Vec3f(0.7,1.0,0.0)};

			lights.push_back(Light(cam.Pos(), colors[rand()&3], 2000.0f * 0.001f * sceneScale));
		}

		MoveCamera(cam, window, speed);

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


		static float animPos = 0;
		if(window.Key(Key_space)) animPos+=0.025f;

		double buildTime = GetTime();
	//		Scene<DBVH> dscene;
	//		dscene.geometry = MakeDBVH(&scene.geometry);
	//		dscene.materials = scene.materials;
		buildTime = GetTime() - buildTime;

		vector<Light> tLights = lightsEnabled?lights:vector<Light>();
			for(int n=0;n<tLights.size();n++)
				tLights[n].pos += Vec3f(sin(animPos+n*n),cos(animPos+n*n),
					sin(animPos-n*n)*cos(animPos+n*n))*speed;
		scene.lights = /*dscene.lights =*/ tLights;
		
		double time = GetTime();
		TreeStats stats;
		stats = Render(scene, cam, image, options, threads);

		time = GetTime() - time;
		window.RenderImage(image);

		double fps = double(unsigned(frmCounter.FPS() * 100)) * 0.01;
		double mrays = double(unsigned(frmCounter.FPS() * stats.GetRays() * 0.0001)) * 0.01;

		font.BeginDrawing(resx,resy);
		font.SetSize(Vec2f(30, 20));
			font.PrintAt(Vec2f(5,  5), stats.GenInfo(resx, resy, time * 1000.0, buildTime * 1000.0));
			font.PrintAt(Vec2f(5, 25), "FPS: ", fps, " MRays/sec:", mrays);
			if(lightsEnabled && lights.size())
				font.PrintAt(Vec2f(5, 45), "Lights: ",lightsEnabled?lights.size() : 0);
			font.PrintAt(Vec2f(5, 85), "prim:", gVals[1], ' ', gVals[2], ' ', gVals[3],
					" sh:", gVals[4], " refl:", gVals[5], ' ', gVals[6], " smart:", gVals[7]);
		font.FinishDrawing();
		window.SwapBuffers();
	}

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
