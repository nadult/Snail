#include <iostream>
#include <fstream>
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

#include "render_opengl.h"
#include "photons.h"

using std::cout;
using std::endl;


static void PrintHelp() {
	printf("Synopsis:    rtracer model_file [options]\nOptions:\n"
			"\t-res x y     - set rendering resolution [1024 1024]\n"
			"\t-fullscreen\n"
			"\t-rebuild     - rebuild BVH tree (using faster algorithm)\n"
			"\t-slowBuild   - rebuild BVH tree (using slower algorithm)\n"
			"\t-flipNormals - flip normals of triangles when rebuilding\n"
			"\t-swapYZ      - swap Y, Z axes when rebuilding\n"
			"\n\n"
			"Interactive control:\n"
			"\tA,W,S,D R,F         - move the camera\n"
			"\tmouse + left button - rotate camera\n"
			"\tmouse wheel         - zoom in/out camera (works only in orbit mode)\n"
			"\tright mouse button  - flip mouse axes (works only in orbit mode)\n"
			"\tshift               - faster movement / zooming\n"
			"\tk   - save image to out/output.tga\n"
			"\to   - toggle reflections\n"
			"\tl   - toggle lights\n"
			"\tj   - add new light in the camera position\n"
			"\tp   - print camera position and save camera configuration\n"
			"\tc   - center camera position in the scene\n"
			"\tF6  - toggle scene complexity visualization\n"
			"\tF7  - toggle advanced shading\n"
			"\tF8  - toggle reflections\n"
			"\tF9  - toggle node tasks visaulization\n"
			"\tF10 - toggle antialiasing\n"
			"\t1-8 - set number of threads to 1 - 8\n"
			"\t9   - set number of threads to 16\n"
			"\t0   - set number of threads to 32\n"
			"\tesc - exit\n\n"
		);
}

static void MoveCamera(OrbitingCamera &cam, GLWindow &window, float speed) {
	if(window.Key(Key_lshift)) speed *= 5.f;
	if(window.MouseKey(0) || window.MouseKey(1)) {
		float flip = window.MouseKey(1)? -1 : 1;

		window.GrabMouse(1);
		float dx = window.MouseMove().x * flip;
		float dy = window.MouseMove().y * flip;

		if(dx) cam.Rotate(-dx * 0.005);
		if(dy) cam.RotateY(dy * 0.005);
	}
	else window.GrabMouse(0);
		
	if(window.Key(Key_left)) cam.Rotate(0.01);
	if(window.Key(Key_right)) cam.Rotate(-0.01);
	
	float zoom = 0;
	zoom = window.MouseMove().z * 4;
	if(window.Key('R')) zoom = 1;
	if(window.Key('F')) zoom = -1;

	if(zoom)
		cam.Zoom(zoom * speed);

	Vec3f move(0, 0, 0);
	Camera tcam = (Camera)cam;

	if(window.Key('W')) move += tcam.front;
	if(window.Key('S')) move -= tcam.front;
	if(window.Key('A')) move -= tcam.right;
	if(window.Key('D')) move += tcam.right;

	cam.target += move * speed;
	cam.pos += move * speed;
}
static void MoveCamera(FPSCamera &cam, GLWindow &window, float speed) {
	if(window.Key(Key_lshift)) speed *= 5.f;
	if(window.MouseKey(0)) {
		window.GrabMouse(1);
		float dx = window.MouseMove().x;
		float dy = window.MouseMove().y;

		if(dx) cam.Rotate(dx * 0.005);
		if(dy) cam.RotateY(dy * 0.005);
	}
	else window.GrabMouse(0);

	Vec3f move(0, 0, 0);
	Camera tcam = (Camera)cam;

	if(window.Key('W')) move += tcam.front;
	if(window.Key('S')) move -= tcam.front;
	if(window.Key('A')) move += tcam.right;
	if(window.Key('D')) move -= tcam.right;
	if(window.Key('R')) move += tcam.up;
	if(window.Key('F')) move -= tcam.up;

	cam.Move(move * speed);
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
	bool flipNormals = 1, swapYZ = 0;
	bool rebuild = 0, slowBuild = 0;
	string texPath = "";

	for(int n = 1; n < argc; n++) {
			 if(string("-res") == argv[n] && n < argc-2) { resx = atoi(argv[n+1]); resy = atoi(argv[n+2]); n += 2; }
		else if(string("-rebuild") == argv[n]) rebuild = 1;
		else if(string("-slowBuild") == argv[n]) { rebuild = 1; slowBuild = 1; }
		else if(string("-threads") == argv[n] && n < argc - 1) { threads=atoi(argv[n+1]); n+=1; }
		else if(string("-fullscreen") == argv[n]) fullscreen = 1;
		else if(string("-flipNormals") == argv[n]) flipNormals = 0;
		else if(string("-swapYZ") == argv[n]) swapYZ = 1;
		else {
			if(argv[n][0] == '-') {
				printf("Unknown option: %s\n",argv[n]);
				exit(0);
			}
			else sceneName = argv[n];
		}
	}
	
	if(texPath == "") {
		texPath = "scenes/" + sceneName;
		int pos = texPath.rfind('/');
		if(pos == string::npos) texPath = "";
		else texPath.resize(pos + 1);
	}

	Scene<BVH> scene;
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
			else if(fileName.find(".list") != string::npos) {
				std::ifstream file(fileName.c_str());

				string elementFileName;
				size_t ntris = 0;

				while(std::getline(file, elementFileName)) {
					elementFileName = "scenes/" + elementFileName;
					BaseScene tmp;
					std::cout << "Loading " << elementFileName << '\n';
					tmp.LoadWavefrontObj(elementFileName);

					for(size_t n = 0; n < tmp.objects.size(); n++) {
						if(tmp.objects[n].tris.size() < 800000) {
							baseScene.objects.push_back(tmp.objects[n]);
							ntris += baseScene.objects.back().tris.size();
						}
					}
					if(ntris > 4000000)
						break;
				}
			}
			else ThrowException("Unrecognized format: ", fileName);

			int tris = 0;
			for(int n = 0; n < baseScene.objects.size(); n++)
				tris += baseScene.objects[n].tris.size();
			printf("Tris: %d\n",tris);

			if(flipNormals) baseScene.FlipNormals();
			if(swapYZ) baseScene.SwapYZ();
		//	for(int n = 0; n < baseScene.objects.size(); n++)
		//		baseScene.objects[n].Repair();
			baseScene.GenNormals();
		//	baseScene.Optimize();
		}
		loadingTime = GetTime() - loadingTime;
		
		double buildTime = GetTime();
		scene.geometry.Construct(baseScene, !slowBuild);
		Saver(string("dump/") + sceneName) & scene.geometry;
		buildTime = GetTime() - buildTime;
		std::cout << "Loading time: " << loadingTime << "  Build time: " << buildTime << '\n';
	}
	scene.geometry.PrintInfo();

	vector<shading::MaterialDesc> matDescs;
	if(sceneName.substr(sceneName.size() - 4) == ".obj") {
		string mtlFile = "scenes/" + sceneName.substr(0, sceneName.size() - 3) + "mtl";
		matDescs = shading::LoadMaterialDescs(mtlFile);
	}
		
	scene.texDict = LoadTextures(matDescs, texPath);
	scene.matDict = shading::MakeMaterials(matDescs, scene.texDict);
		
	scene.UpdateMaterials();
	scene.geometry.UpdateMaterialIds(scene.GetMatIdMap());

	float sceneScale; {
		Vec3f size = scene.geometry.GetBBox().Size();
		sceneScale = Length(size);
	}
	Vec3f sceneCenter = scene.geometry.GetBBox().Center();

	GLWindow window(resx, resy, fullscreen);
	Font font;

	gfxlib::Texture image(resx, resy, gfxlib::TI_R8G8B8);
	vector<Light> lights;// = GenLights();
			
	FPSCamera cam;
	if(!camConfigs.GetConfig(string(sceneName),cam))
		cam.SetPos(sceneCenter);
	OrbitingCamera ocam;

	bool lightsEnabled = 1;
	bool staticEnabled = 0, orbiting = 0;
	float speed; {
	//	scene.geometry.Construct(builder.ExtractElements());
		Vec3f size = scene.geometry.GetBBox().Size();
		speed = (size.x + size.y + size.z) * 0.0025f;
	}

	OGLRenderer *oglRenderer = 0;
	bool oglRendering = 0;

	FrameCounter frmCounter;
	float lastFrameTime = 0.0f;
	double lastTime = GetTime();

	vector<Photon> photons;
	if(!oglRenderer) oglRenderer = new OGLRenderer(scene);
	double photonGenTime = 0.0;

	while(window.PollEvents()) {
		frmCounter.NextFrame();

		if(window.KeyUp(Key_esc)) break;
		if(window.KeyDown('K')) Saver("out/output.dds") & image;

		if(window.KeyDown('C')) {
			if(orbiting) ocam.Reset(sceneCenter, sceneScale);
			else cam.SetPos(sceneCenter);
		}
		if(window.KeyDown('P')) {
			camConfigs.AddConfig(string(sceneName),cam);
			Saver("cameras.dat") & camConfigs;
			cam.Print();
		}
		if(window.KeyDown('O')) {
			if(orbiting) cam.SetPos(ocam.pos);
			else ocam.Reset(cam.pos, -sceneScale);
			orbiting ^= 1;
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
			Vec3f colors[] = {
			//	Vec3f(1.0, 1.0, 1.0),
				Vec3f(0.2, 0.5, 1.0),
				Vec3f(1.0, 0.5, 0.2),
			//	Vec3f(0.7, 1.0, 0.0),
			};

			Vec3f pos = (orbiting?(Camera)ocam : (Camera)cam).pos;
			lights.push_back(Light(pos, colors[rand() % countof(colors)], 2000.0f * 0.001f * sceneScale));
		}

		if(window.KeyDown('H')) {
			oglRendering ^= 1;
			if(oglRendering && !oglRenderer) {
				try {
					oglRenderer = new OGLRenderer(scene);
				}
				catch(const Exception &ex) {
					std::cout << ex.what();
					oglRendering = false;
				}
			}
		}

		float tspeed = speed * lastFrameTime * 20.0f;
		lastFrameTime = GetTime() - lastTime;
		lastTime = GetTime();

		if(orbiting)
			MoveCamera(ocam, window, tspeed);
		else
			MoveCamera(cam, window, tspeed);

		for(int n = 1; n <= 8; n++) if(window.Key('0' + n))
				{ threads = n; printf("Threads: %d\n", threads); }

		if(window.KeyDown(Key_f1)) { gVals[0]^=1; printf("Val 0 %s\n", gVals[0]?"on" : "off"); }
		if(window.KeyDown(Key_f2)) { gVals[1]^=1; printf("Val 1 %s\n", gVals[1]?"on" : "off"); }
		if(window.KeyDown(Key_f3)) { gVals[2]^=1; printf("Val 2 %s\n", gVals[2]?"on" : "off"); }
		if(window.KeyDown(Key_f4)) { gVals[3]^=1; printf("Val 3 %s\n", gVals[3]?"on" : "off"); }
		if(window.KeyDown(Key_f5)) { gVals[4]^=1; printf("Photon tracing %s\n", gVals[4]?"on" : "off"); }
		if(window.KeyDown(Key_f6)) { gVals[5]^=1; printf("Scene complexity visualization %s\n",gVals[5]?"on":"off"); }
		if(window.KeyDown(Key_f7)) { gVals[6]^=1; printf("Advanced shading %s\n",gVals[6]?"on":"off"); }
		if(window.KeyDown(Key_f8)) { gVals[7]^=1; printf("Reflections 7 %s\n",gVals[7]?"on":"off"); }
		if(window.KeyDown(Key_f9)) { gVals[8]^=1; printf("Node tasks visualization 8 %s\n",gVals[8]?"on":"off"); }
		if(window.KeyDown(Key_f10)) { gVals[9]^=1; printf("Antialiasing 4x %s\n",gVals[9]?"on":"off"); }
		if(window.KeyDown(Key_f1)) { gVals[0]^=1; printf("Traversing from 8x8: %s\n", gVals[0]?"on" : "off"); }

		static float animPos = 0;
		if(window.Key(Key_space)) animPos+=0.025f;

		double buildTime = GetTime();
			Scene<DBVH> dscene;
		//	dscene.geometry = MakeDBVH(&scene.geometry);
		//	dscene.materials = scene.materials;
		buildTime = GetTime() - buildTime;

		vector<Light> tLights = lightsEnabled?lights:vector<Light>();
		//	for(int n=0;n<tLights.size();n++)
		//		tLights[n].pos += Vec3f(sin(animPos+n*n),cos(animPos+n*n),
		//			sin(animPos-n*n)*cos(animPos+n*n)) * tspeed;
		scene.lights = dscene.lights = tLights;
		
		double time = GetTime(); 
		TreeStats stats; {
			Camera camera = orbiting?(Camera)ocam : (Camera)cam;

			if(oglRendering) {
				oglRenderer->BeginDrawing(camera, 60.0f, float(resx) / float(resy), 1);
				oglRenderer->Draw();
				oglRenderer->DrawPhotons(photons, 0);
				oglRenderer->FinishDrawing();
			}
			else {
				stats = Render(scene, camera, image, options, threads);
				bool updated = 0;

				if(gVals[4]) {
					photonGenTime = GetTime();
					TracePhotons(photons, scene, 4 * 1024 * 1024);
					photonGenTime = GetTime() - photonGenTime;
					gVals[4] = 0;
					updated = 1;
				}
				window.RenderImage(image);

				oglRenderer->BeginDrawing(camera, 60.0f, float(resx) / float(resy), 0);
				oglRenderer->DrawPhotons(photons, updated);
				oglRenderer->FinishDrawing();
			}
		}	

		time = GetTime() - time;

		double fps = double(unsigned(frmCounter.FPS() * 100)) * 0.01;
		double mrays = double(unsigned(frmCounter.FPS() * stats.GetRays() * 0.0001)) * 0.01;
		double ptime = double(unsigned(photonGenTime * 100.0)) * 0.01;

		font.BeginDrawing(resx,resy);
		font.SetSize(Vec2f(30, 20));
		if(oglRendering) {
			font.PrintAt(Vec2f(5, 25), "FPS: ", fps, " (opengl rendering)");
		}
		else {
			font.PrintAt(Vec2f(5,  5), stats.GenInfo(resx, resy, time * 1000.0, buildTime * 1000.0));
			font.PrintAt(Vec2f(5, 25), "FPS: ", fps, " MRays/sec:", mrays);
			font.PrintAt(Vec2f(5, 45), photons.size() / 1024, "k photons generated in ", ptime);
			if(lightsEnabled && lights.size())
				font.PrintAt(Vec2f(5, 65), "Lights: ",lightsEnabled?lights.size() : 0);
			font.PrintAt(Vec2f(5, 85), "prim:", gVals[1], ' ', gVals[2], ' ', gVals[3],
					" sh:", gVals[4], " refl:", gVals[5], ' ', gVals[6], " smart:", gVals[7]);
		}

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
