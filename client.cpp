#include "pch.h"
#include <iostream>
#include "camera.h"

#include "gl_window.h"
#include "scene.h"
#include "font.h"
#include "frame_counter.h"
#include "compression.h"
#include "comm.h"
#include "tree_stats.h"

#include <GL/gl.h>


using comm::Socket;
using comm::PSocket;
using comm::Pod;

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
			"\t-threads n   - set threads number to n\n"
			"\t-nodes n     - set nodes number to n\n"
			"\t-instances n	- draw N istances of given model\n"
			"\t-host hname  - connect to host hname [localhost]\n"
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

void SendFrameRequest(PSocket sock, Camera cam, vector<Light> &lights, int threads,
						int nInstances, float dynAnimPos, bool finish) {
	int size = lights.size();

	sock << finish << Pod(cam) << size;
	sock << comm::Data(&lights[0], size * sizeof(Light));
	sock << Pod(gVals) << threads << nInstances << dynAnimPos;
}

static int client_main(int argc, char **argv) {
	printf("Snail v0.21 by nadult\n");
	if(argc >= 2 && string("--help") == argv[1]) {
		PrintHelp();
		return 0;
	}
	else printf("type './rtracer --help' to get some help\n");

	CameraConfigs camConfigs;
	try { Loader("cameras.dat") & camConfigs; } catch(...) { }

	int resx = 1024, resy = 1024;
	bool fullscreen = 0;
	int threads = 2, nNodes = -1;
	const char *modelFile = "foot.obj";
	const char *host = "localhost";

//	Options options;
	bool flipNormals = 1, swapYZ = 0;
	int rebuild = 0;
	int nInstances = 1;

	for(int n=1;n<argc;n++) {
			 if(string("-res")==argv[n]&&n<argc-2) { resx=atoi(argv[n+1]); resy=atoi(argv[n+2]); n+=2; }
		else if(string("-rebuild") == argv[n]) rebuild = 1;
		else if(string("-slowBuild") == argv[n]) rebuild = 2;
		else if(string("-threads") == argv[n] && n < argc - 1) { threads=atoi(argv[n+1]); n+=1; }
		else if(string("-fullscreen") == argv[n]) fullscreen = 1;
		else if(string("-flipNormals") == argv[n]) flipNormals = 0;
		else if(string("-swapYZ") == argv[n]) swapYZ = 1;
		else if(string("-nodes") == argv[n]&&n<argc-1) { nNodes=atoi(argv[n+1]); n++; }
		else if(string("-host") == argv[n]) { host=argv[n+1]; n++; }
		else if(string("-instances") == argv[n]) { nInstances = atoi(argv[n + 1]); n += 1; }
		else {
			if(argv[n][0] == '-') {
				printf("Unknown option: %s\n",argv[n]);
				exit(0);
			}
			else modelFile = argv[n];
		}
	}

	GLWindow window(resx, resy, fullscreen);
	window.SetTitle("Snail client v0.21");

	resx = ((resx + blockWidth  - 1) / blockWidth ) * blockWidth;
	resy = ((resy + blockHeight - 1) / blockHeight) * blockHeight;
	Font font;

	gfxlib::Texture image(resx, resy, gfxlib::TI_R8G8B8);
	vector<Light> lights;
			
	bool lightsEnabled = 1;

	FrameCounter frmCounter;
		
	Socket socket;
	socket.Connect(host, "20002");
	socket.NoDelay(1);
	PSocket sock(socket);

	Vec3f sceneCenter, sceneSize;
	{
		comm::LoadNewModel lm;
		lm.name = modelFile; lm.resx = resx; lm.resy = resy;
		lm.rebuild = rebuild; lm.flipNormals = flipNormals;
		lm.swapYZ = swapYZ; lm.nNodes = nNodes;
		sock << lm;
		printf("Waiting for server..\n");
		sock >> sceneCenter >> sceneSize;
	}
	float speed = (sceneSize.x + sceneSize.y + sceneSize.z) * 0.0025f;
	float sceneScale = Length(sceneSize);

	FPSCamera cam;
	if(!camConfigs.GetConfig(string(modelFile),cam))
		cam.SetPos(sceneCenter);
	OrbitingCamera ocam;

	bool finish = 0, orbiting = 0;
	SendFrameRequest(sock, (Camera)cam, lights, threads, nInstances, 0.0f, finish);
	double frameTime = getTime();

	vector<DecompressBuffer> buffers(2);

	double fpsMin = constant::inf, fpsMax = 0, fpsSum = 0;
	long long nFrames = 0, nRays = 0;
	double startTime = getTime();

	while(!finish) {
		if(!window.PollEvents() || window.KeyUp(Key_esc)) finish = 1;
		frmCounter.NextFrame();
		
		if(window.KeyDown('K'))
			Saver("output.dds") & image;
		if(window.KeyDown('C')) {
			if(orbiting) ocam.Reset(sceneCenter, sceneScale);
			else cam.SetPos(sceneCenter);
		}
		if(window.KeyDown('P')) {
			camConfigs.AddConfig(string(modelFile),cam);
			Saver("cameras.dat") & camConfigs;
			cam.Print();
		}
		if(window.KeyDown('O')) {
			if(orbiting) cam.SetPos(ocam.pos);
			else ocam.Reset(cam.pos, -sceneScale);
			orbiting ^= 1;
		}
		if(window.KeyDown('X')) {
			printf("Clearing stats\n");
			fpsMin = constant::inf;
			fpsMax = fpsSum = 0;
			nFrames = nRays = 0;
			startTime = getTime();
		}
		if(window.KeyDown('Z')) {
			printf("Stats:\n");
			double workTime = getTime() - startTime;
			double fpsAvg = fpsSum / double(nFrames);
			double mraysPerSec = (double(nRays) / workTime) / 1000000;
			printf("Min FPS: %f\nMax FPS: %f\nAvg FPS: %f\nMRays/sec: %f\n",
					fpsMin, fpsMax, fpsAvg, mraysPerSec);
		}

		if(window.KeyDown('L')) {
			printf("Lights %s\n", lightsEnabled? "disabled" : "enabled");
			lightsEnabled ^= 1;
		}
		if(window.KeyDown('J')) {
			Vec3f colors[4] = {
				Vec3f(1, 1, 1), Vec3f(0.2, 0.5, 1), Vec3f(0.5, 1, 0.2), Vec3f(0.7, 1.0, 0.0) };

			Vec3f pos = (orbiting?(Camera)ocam : (Camera)cam).pos;
			srand(time(0));
			lights.push_back(Light(pos, colors[rand() & 3], 1000.0f * 0.001f * sceneScale));
		}

		if(orbiting)
			MoveCamera(ocam, window, speed);
		else
			MoveCamera(cam, window, speed);

		int lastThreads = threads;
		for(int n = 0; n <= 9; n++) if(window.Key('0' + n)) {
			int newThreads = n == 0?32 : n == 9? 16 : n;
			if(newThreads != lastThreads) {
				threads = newThreads;
				printf("Threads: %d\n", threads);
				break;
			}
		}

		if(window.KeyDown(Key_f1)) { gVals[0]^=1; printf("Val 0 %s\n", gVals[0]?"on" : "off"); }
		if(window.KeyDown(Key_f2)) { gVals[1]^=1; printf("Val 1 %s\n", gVals[1]?"on" : "off"); }
		if(window.KeyDown(Key_f3)) { gVals[2]^=1; printf("Val 2 %s\n", gVals[2]?"on" : "off"); }
		if(window.KeyDown(Key_f4)) { gVals[3]^=1; printf("Val 3 %s\n", gVals[3]?"on" : "off"); }
		if(window.KeyDown(Key_f5)) { gVals[4]^=1; printf("Val 4 %s\n", gVals[4]?"on" : "off"); }
		if(window.KeyDown(Key_f6)) { gVals[5]^=1; printf("Scene complexity visualization %s\n",gVals[5]?"on":"off"); }
		if(window.KeyDown(Key_f7)) { gVals[6]^=1; printf("Advanced shading %s\n",gVals[6]?"on":"off"); }
		if(window.KeyDown(Key_f8)) { gVals[7]^=1; printf("Reflections 7 %s\n",gVals[7]?"on":"off"); }
		if(window.KeyDown(Key_f9)) { gVals[8]^=1; printf("Node tasks visualization 8 %s\n",gVals[8]?"on":"off"); }
		if(window.KeyDown(Key_f10)) { gVals[9]^=1; printf("Antialiasing 4x %s\n",gVals[9]?"on":"off"); }

		static float animPos = 0;
		if(window.Key(Key_space)) animPos += 0.025f;

		vector<Light> tLights = lightsEnabled?lights:vector<Light>();
	//		for(int n=0;n<tLights.size();n++)
	//			tLights[n].pos += Vec3f(sin(animPos+n*n),cos(animPos+n*n),
	//				sin(animPos-n*n)*cos(animPos+n*n)) * speed * 100.0f;

		int nBytes = 0, nBuffers = 0, numNodes;
		double buildTime, decompressTime = 0, renderTimes[32];
		TreeStats stats;
		SendFrameRequest(sock, orbiting?(Camera)ocam : (Camera)cam, tLights, threads, nInstances, animPos, finish);

		while(true) {
			DecompressBuffer &buffer = buffers[nBuffers++];

			sock >> buffer.comprSize;
			if(buffer.comprSize == 0) {
				nBuffers--;
				break;
			}

			nBytes += Abs(buffer.comprSize);
			if(buffer.comprData.size() < Abs(buffer.comprSize))
				buffer.comprData.resize(Abs(buffer.comprSize));
			sock >> comm::Data(&buffer.comprData[0], Abs(buffer.comprSize));

			if(nBuffers == buffers.size()) {
				double tTime = getTime();
				DecompressParts(image, buffers, nBuffers, 2);
				decompressTime += getTime() - tTime;
				nBuffers = 0;
			}
		}

		{
			double tTime = getTime();
			DecompressParts(image, buffers, nBuffers, 2);
			decompressTime += getTime() - tTime;
		}
		sock >> Pod(stats) >> buildTime >> numNodes >> Pod(renderTimes);

//		glClear(GL_COLOR_BUFFER_BIT);
//		TODO: jest duzy spadek wydajnosci miedzy 1376x1024 a 1344x1024
		window.RenderImage(image, false);

		double fps = double(unsigned(frmCounter.FPS() * 100)) * 0.01;
		double mrays = double(unsigned(frmCounter.FPS() * stats.GetRays() * 0.0001)) * 0.01;

		nFrames++;
		nRays += stats.GetRays();
		fpsMin = Min(fpsMin, frmCounter.FPS());
		fpsMax = Max(fpsMax, frmCounter.FPS());
		fpsSum += frmCounter.FPS();

		font.BeginDrawing(resx, resy);
		font.SetSize(Vec2f(30, 20));
			font.SetPos(Vec2f(5, 5));
			font.Print(stats.GenInfo(resx, resy, (getTime() - frameTime) * 1000.0, buildTime * 1000.0f));
			frameTime = getTime();
			font.SetPos(Vec2f(5, 25));
			char text[256];
			snprintf(text, sizeof(text), "FPS: ", fps, " MRays/sec:", mrays, " KBytes/frame:", nBytes / 1024,
					" Dec.time:", double((int)(decompressTime * 100000.0)) * 0.01);
			font.Print(text);
			
			for(int n = 0; n < Min(32, numNodes); n++) {
				snprintf(text, sizeof(text), "%.0f", renderTimes[n] * 1000);
				font.SetPos(Vec2f(5 + n * 32, 45));
				font.Print(text);
			}
			if(lightsEnabled && lights.size()) {
				font.SetPos(Vec2f(5, 65));
				snprintf(text, sizeof(text), "Lights: ",lightsEnabled?lights.size() : 0);
				font.Print(text);
			}
		font.FinishDrawing();
		window.SwapBuffers();
	}

	double workTime = getTime() - startTime;
	double fpsAvg = fpsSum / double(nFrames);
	double mraysPerSec = (double(nRays) / workTime) / 1000000;

	printf("Min FPS: %f\nMax FPS: %f\nAvg FPS: %f\nMRays/sec: %f\n",
			fpsMin, fpsMax, fpsAvg, mraysPerSec);
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
