#include "pch.h"
#include <iostream>
#include "camera.h"

#include "gl_window.h"
#include "formats/loader.h"
#include "base_scene.h"
#include "bvh/tree.h"
#include "render.h"

#include "scene.h"
#include "font.h"
#include "frame_counter.h"

#include "compression.h"
#include "comm.h"

#include <GL/gl.h>

using comm::Socket;
using comm::Pod;

using std::cout;
using std::endl;

static void PrintHelp() {
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

static vector<Light> GenLights(float scale = 1.0f, float power = 1000.0f) {
	vector<Light> out;

	float pos = float(gVals[5]) * 0.01f;

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
		window.GrabMouse(1);
		float dx = window.MouseMove().x;
		float dy = window.MouseMove().y;

		if(dx) cam.Rotate(-dx * 0.005);
		if(dy) cam.RotateY(dy * 0.005);
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

void SendFrameRequest(Socket &socket, Camera &cam, vector<Light> &lights,
					int threads, bool finish) {
	int size = lights.size();

	socket << Pod(finish) << Pod(cam) << Pod(size);
	socket << comm::Data(&lights[0], size * sizeof(Light));
	socket << Pod(gVals) << Pod(threads);
}

static int client_main(int argc, char **argv) {
	printf("Snail v0.20 by nadult\n");
	if(argc>=2&&string("--help")==argv[1]) {
		PrintHelp();
		return 0;
	}
	else printf("type './rtracer --help' to get some help\n");

	CameraConfigs camConfigs;
	try { Loader("scenes/cameras.dat") & camConfigs; } catch(...) { }

	int resx=1024, resy=1024;
	bool fullscreen = 0;
	int threads = 2;
	const char *modelFile = "foot.obj";
	const char *host = "blader";

	Options options;
	bool flipNormals = 1;
	bool rebuild = 0, slowBuild = 0;
	string texPath = "scenes/";

	for(int n=1;n<argc;n++) {
			 if(string("-res")==argv[n]&&n<argc-2) { resx=atoi(argv[n+1]); resy=atoi(argv[n+2]); n+=2; }
		else if(string("-rebuild") == argv[n]) { rebuild = 1; }
		else if(string("-slowBuild") == argv[n]) { rebuild = 1; slowBuild = 1; }
		else if(string("-threads")==argv[n]&&n<argc-1) { threads=atoi(argv[n+1]); n+=1; }
		else if(string("-fullscreen")==argv[n]) { fullscreen=1; }
		else if(string("+flipNormals")==argv[n]) flipNormals=1;
		else if(string("-flipNormals")==argv[n]) flipNormals=0;
		else if(string("-texPath")==argv[n]) { texPath=argv[n+1]; n++; }
		else if(string("-host")==argv[n]) { host=argv[n+1]; n++; }
		else {
			if(argv[n][0] == '-') {
				printf("Unknown option: %s\n",argv[n]);
				exit(0);
			}
			else modelFile = argv[n];
		}
	}

	if(rebuild) {
		Scene<StaticTree> staticScene;

		printf("Loading...\n");
		double buildTime = GetTime();
		BaseScene baseScene; {
			string fileName = string("scenes/") + modelFile;
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
		
		staticScene.geometry.Construct(baseScene.ToCompactTris(), baseScene.matNames, !slowBuild);
	//	staticScene.geometry.Construct(baseScene.ToTriangleVector());
		Saver(string("dump/") + modelFile) & staticScene.geometry;
		buildTime = GetTime() - buildTime;
		std::cout << "Build time: " << buildTime << '\n';
	
		staticScene.geometry.PrintInfo();
	}


	GLWindow window(resx, resy, fullscreen);
	Font font;

	gfxlib::Texture image(resx, resy, gfxlib::TI_R8G8B8);
	vector<Light> lights;
			
	bool lightsEnabled = 1;

	FrameCounter frmCounter;
		
	Socket socket;
	socket.Connect(host, "20002");

	Vec3f sceneCenter, sceneSize;
	{
		char model[256]; snprintf(model, 256, "%s", modelFile);
		socket << Pod(resx) << Pod(resy) << Pod(model);		
		printf("Waiting for server..\n");
		socket >> Pod(sceneCenter) >> Pod(sceneSize);
	}
	float speed = (sceneSize.x + sceneSize.y + sceneSize.z) * 0.0025f;
	float sceneScale = Length(sceneSize);

	Camera cam;
	if(!camConfigs.GetConfig(string(modelFile),cam))
		cam.SetPos(sceneCenter);

	bool finish = 0, displayEnabled = 1;
	SendFrameRequest(socket, cam, lights, threads, finish);
	vector<CompressedPart> parts;
	double frameTime = GetTime();

	while(!finish) {
		if(!window.PollEvents() || window.KeyUp(Key_esc)) finish = 1;
		frmCounter.NextFrame();
		
		if(window.KeyDown('K'))
			Saver("out/output.dds") & image;
		if(window.KeyDown('I'))
			options.rdtscShader ^= 1;
		if(window.KeyDown('C'))
			cam.SetPos(sceneCenter);
		if(window.KeyDown('P')) {
			camConfigs.AddConfig(string(modelFile),cam);
			Saver("scenes/cameras.dat") & camConfigs;
			cam.Print();
		}
		if(window.KeyDown('X')) displayEnabled ^= 1;

		if(window.KeyDown('L')) {
			printf("Lights %s\n", lightsEnabled? "disabled" : "enabled");
			lightsEnabled ^= 1;
		}
		if(window.KeyDown('J')) {
			Vec3f colors[4] = {
				Vec3f(1,1,1), Vec3f(0.2,0.5,1),Vec3f(0.5,1,0.2),Vec3f(0.7,1.0,0.0) };

			srand(time(0));
			lights.push_back(Light(cam.Pos(), colors[rand()&3], 1000.0f * 0.001f * sceneScale));
		}

		MoveCamera(cam, window, speed);

		for(int n = 0; n <= 9; n++) if(window.Key('0' + n))
				{ threads = n == 0?32 : n == 9? 16 : n; printf("Threads: %d\n", threads); }

		if(window.KeyDown(Key_f1)) { gVals[0]^=1; printf("Traversing from 8x8: %s\n", gVals[0]?"on" : "off"); }
		if(window.KeyDown(Key_f2)) { gVals[1]^=1; printf("Val 2 %s\n",gVals[1]?"on":"off"); }
		if(window.KeyDown(Key_f3)) { gVals[2]^=1; printf("Val 3 %s\n",gVals[2]?"on":"off"); }
		if(window.KeyDown(Key_f4)) { gVals[3]^=1; printf("Val 4 %s\n",gVals[3]?"on":"off"); }
		if(window.KeyDown(Key_f5)) { gVals[4]^=1; printf("Toggled shading\n"); }
		if(window.KeyDown(Key_f6)) { gVals[5]^=1; printf("Val 5 %s\n",gVals[5]?"on":"off"); }
		if(window.KeyDown(Key_f7)) { gVals[6]^=1; printf("Val 6 %s\n",gVals[6]?"on":"off"); }
		if(window.KeyDown(Key_f8)) { gVals[7]^=1; printf("Val 7 %s\n",gVals[7]?"on":"off"); }
		if(window.KeyDown(Key_f9)) { gVals[8]^=1; printf("Val 8 %s\n",gVals[8]?"on":"off"); }
		if(window.KeyDown(Key_f10)) { gVals[9]^=1; printf("Val 9 %s\n",gVals[9]?"on":"off"); }

		static float animPos = 0;
		if(window.Key(Key_space)) animPos += 0.025f;

		vector<Light> tLights = lightsEnabled?lights:vector<Light>();
	//		for(int n=0;n<tLights.size();n++)
	//			tLights[n].pos += Vec3f(sin(animPos+n*n),cos(animPos+n*n),
	//				sin(animPos-n*n)*cos(animPos+n*n)) * speed * 100.0f;
		int w = image.Width(), h = image.Height();

		int nBytes = 0, numNodes;
		double buildTime, renderTimes[32];
		TreeStats stats; if(displayEnabled) {
			int nPixels = w * h, received = 0;
			SendFrameRequest(socket, cam, tLights, threads, finish);

			int nParts = 0;
			while(received < nPixels) {
				if(parts.size() < nParts + 1)
					parts.push_back(CompressedPart());

				CompressedPart &part = parts[nParts++];
				socket >> Pod(part.info);
				if(part.data.size() < part.info.size)
					part.data.resize(part.info.size);
				socket >> comm::Data(&part.data[0], part.info.size);
				nBytes += part.info.size;
				received += part.info.w * part.info.h;

				if(nParts >= 8) {
					DecompressParts(image, parts, nParts, 2);
					nParts = 0;
				}
			}
				
			DecompressParts(image, parts, nParts, 2);
			socket >> Pod(stats) >> Pod(buildTime) >> Pod(numNodes) >> Pod(renderTimes);
		}

		window.RenderImage(image);

		double fps = double(unsigned(frmCounter.FPS() * 100)) * 0.01;
		double mrays = double(unsigned(frmCounter.FPS() * stats.GetRays() * 0.0001)) * 0.01;

		font.BeginDrawing(resx,resy);
		font.SetSize(Vec2f(30, 20));
			font.PrintAt(Vec2f(5,  5), stats.GenInfo(resx, resy, (GetTime() - frameTime) * 1000.0, buildTime * 1000.0f));
			frameTime = GetTime();
			font.PrintAt(Vec2f(5, 25), "FPS: ", fps, " MRays/sec:", mrays, " KBytes/frame:", nBytes / 1024);
			font.PrintAt(Vec2f(5, 45), "prim:", gVals[1], ' ', gVals[2], ' ', gVals[3],
					" sh:", gVals[4], " refl:", gVals[5], ' ', gVals[6], " smart:", gVals[7]);
			for(int n = 0; n < Min(32, numNodes); n++) {
				char text[32]; snprintf(text, sizeof(text), "%.0f", renderTimes[n] * 1000);
				font.PrintAt(Vec2f(5 + n * 32, 65), text);
			}
			if(lightsEnabled && lights.size())
				font.PrintAt(Vec2f(5, 85), "Lights: ",lightsEnabled?lights.size() : 0);
		font.FinishDrawing();
		window.SwapBuffers();
	}

	return 0;
}

int main(int argc,char **argv) {
	try {
		return client_main(argc, argv);
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
