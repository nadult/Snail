#include <iostream>
#include <fstream>
#include <cstring>
#include <algorithm>
#include "camera.h"

#include "gl_window.h"
#include "frame_counter.h"

#include "font.h"
#include "frame_counter.h"

using std::cout;
using std::endl;


struct DICOM {
	struct Slice {
		void Serialize(Serializer &sr) {
			char zeros[128] = {0,};
			sr.Data(zeros, sizeof(zeros));

			sr.Signature("DICM", 4);
			InputAssert(sr.IsLoading());

			int samples = 1;
			int bits = 8;
			width = height = 0;
			data.clear();

			while(sr.Pos() < sr.Size()) {
				u16 type, subType;
				u16 format, size16;
				u32 size;

				sr(type, subType, format, size16);
				if(size16 == 0 && (type != 0x08 && type != 0x20))
					sr & size;
				else
					size = size16;

				u32 end = sr.Pos() + size;

				if(type == 0x28) {
					u16 tmp; sr & tmp;
					if(subType == 0x10)
						width = tmp;
					else if(subType == 0x11)
						height = tmp;
					else if(subType == 0x08)
						samples = tmp;
					else if(subType == 0x100)
						bits = tmp;

				}
				else if(type == 0x7fe0 && subType == 0x10) {
					data.resize(size / 2);
					sr.Data(&data[0], size);
				}

				sr.Seek(end);
			}

			InputAssert(bits == 16 && samples == 1);
			InputAssert(data.size() * sizeof(data[0]) == width * height * (bits / 8));
		}
	
		int width, height;
		vector<u16> data;
	};

	void Load(const char *folder) {
		vector<string> files = FindFiles(folder, ".dcm", false);

		std::sort(files.begin(), files.end());

		Slice slice; Loader(files[0]) & slice;
		width = slice.width;
		height = slice.height;
		depth = files.size();

		data.resize(width * height * depth);
		memcpy(&data[0], &slice.data[0], width * height * sizeof(data[0]));

		for(int n = 1; n < files.size(); n++) {
			Loader(files[n]) & slice;
			InputAssert(width == slice.width);
			InputAssert(height == slice.height);
			memcpy(&data[width * height * n], &slice.data[0], width * height * sizeof(data[0]));
			printf("."); fflush(stdout);
		}
	}

	void Blit(gfxlib::Texture &img, int slice) const {
		int w = Min(width, img.Width()), h = Min(height, img.Height());
		InputAssert(img.GetFormat() == gfxlib::TI_R8G8B8);

		for(int y = 0; y < h; y++) {
			u8 *dst = (u8*)img.DataPointer(0);
			const u16 *src = &data[slice * width * height + y * width];
			dst += img.Pitch() * y;

			for(int x = 0; x < w; x++) {
				int value = (src[x] >> 7) * 2;
				value = Min(value, 255);
				dst[x * 3 + 0] = value;
				dst[x * 3 + 1] = value;
				dst[x * 3 + 2] = value;
			}
		}
	}

	vector<u16> data;
	int width, height, depth;
};


static void PrintHelp() {
	printf("Synopsis:    rtracer model_file [options]\nOptions:\n"
			"\t-res x y       - set rendering resolution [1024 1024]\n"
			"\t-fullscreen\n"
			"\t-flipNormals   - flip normals of triangles when rebuilding\n"
			"\t-swapYZ        - swap Y, Z axes when rebuilding\n"
			"\t-noSah         - don't use SAH when rebuilding\n"
			"\t-noShadingData - build tree without shading data (saves memory)\n"
			"\t-noDump        - don't save generated tree to disk\n"
			"\t-instances n   - draw N istances of given model\n"
			"\n\n"
			"Interactive control:\n"
			"\tA,W,S,D R,F         - move the camera\n"
			"\tmouse + left button - rotate camera\n"
			"\tmouse wheel         - zoom in/out camera (works only in orbit mode)\n"
			"\tright mouse button  - flip mouse axes (works only in orbit mode)\n"
			"\tshift               - faster movement / zooming\n"
			"\tk   - save image to out/output.tga\n"
			"\to   - toggle orbit mode\n"
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

static int tmain(int argc, char **argv) {
	printf("Snail v0.20 by nadult\n");
	if(argc>=2 && string("--help") == argv[1]) {
		PrintHelp();
		return 0;
	}
	else printf("type './rtracer --help' to get some help\n");

	CameraConfigs camConfigs;
	try { Loader("scenes/cameras.dat") & camConfigs; } catch(...) { }

	int resx = 1024, resy = 1024;
	bool fullscreen = 0;
	int threads = 2;
	string sceneName = "sponza.obj";

	bool flipNormals = 1, swapYZ = 0;
	string texPath = "";
	int rebuild = 0, buildFlags = 8;

	int nInstances = 1;
	DICOM dicom;
	dicom.Load("/mnt/data/zatoki/dicom/");

	for(int n = 1; n < argc; n++) {
			 if(string("-res") == argv[n] && n < argc-2) { resx = atoi(argv[n+1]); resy = atoi(argv[n+2]); n += 2; }
		else if(string("-rebuild") == argv[n]) rebuild = 1;
		else if(string("-threads") == argv[n] && n < argc - 1) { threads = atoi(argv[n + 1]); n += 1; }
		else if(string("-fullscreen") == argv[n]) fullscreen = 1;
		else if(string("-flipNormals") == argv[n]) flipNormals = 0;
		else if(string("-swapYZ") == argv[n]) swapYZ = 1;
		else if(string("-noDump") == argv[n]) buildFlags &= ~8;
		else if(string("-instances") == argv[n]) { nInstances = atoi(argv[n + 1]); n += 1; }
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

	float sceneScale = 1.0f;
	Vec3f sceneCenter(0.0f, 0.0f, 0.0f);

	GLWindow window(resx, resy, fullscreen);
	window.SetTitle("DICOM viewer");
	Font font;

	gfxlib::Texture image(550, 550, gfxlib::TI_R8G8B8);
			
	FPSCamera cam;
	if(!camConfigs.GetConfig(string(sceneName),cam))
		cam.SetPos(sceneCenter);
	OrbitingCamera ocam;

	bool lightsEnabled = 1;
	bool staticEnabled = 0, orbiting = 0;
	float speed = 1.0f;
	int slice = 0;

	FrameCounter frmCounter;
	float lastFrameTime = 0.0f;
	double lastTime = GetTime();

	while(window.PollEvents()) {
		frmCounter.NextFrame();

		if(window.KeyUp(Key_esc)) break;
		if(window.KeyDown('K')) Saver("out/output.dds") & image;

		if(window.KeyDown('C')) {
			if(orbiting) ocam.Reset(sceneCenter, sceneScale);
			else cam.SetPos(sceneCenter);
			cam.ang = 0;
			cam.pitch = 0;
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

		slice += window.MouseMove().z;
		slice = Clamp(slice, 0, dicom.depth - 1);

		float tspeed = speed * lastFrameTime * 20.0f;
		lastFrameTime = GetTime() - lastTime;
		lastTime = GetTime();

		if(orbiting)
			MoveCamera(ocam, window, tspeed);
		else
			MoveCamera(cam, window, tspeed);

		for(int n = 1; n <= 8; n++) if(window.KeyDown('0' + n))
				{ threads = n; printf("Threads: %d\n", threads); }

		if(window.KeyDown(Key_f1)) { gVals[0]^=1; printf("Val 0 %s\n", gVals[0]?"on" : "off"); }
		if(window.KeyDown(Key_f2)) { gVals[1]^=1; printf("Val 1 %s\n", gVals[1]?"on" : "off"); }
		if(window.KeyDown(Key_f3)) { gVals[2]^=1; printf("Val 2 %s\n", gVals[2]?"on" : "off"); }
		if(window.KeyDown(Key_f4)) { gVals[3]^=1; printf("Photons visible %s\n", gVals[3]?"on" : "off"); }
		if(window.KeyDown(Key_f5)) { gVals[4]^=1; printf("Photon tracing %s\n", gVals[4]?"on" : "off"); }
		if(window.KeyDown(Key_f6)) { gVals[5]^=1; printf("Scene complexity visualization %s\n",gVals[5]?"on":"off"); }
		if(window.KeyDown(Key_f7)) { gVals[6]^=1; printf("Advanced shading %s\n",gVals[6]?"on":"off"); }
		if(window.KeyDown(Key_f8)) { gVals[7]^=1; printf("Reflections 7 %s\n",gVals[7]?"on":"off"); }
		if(window.KeyDown(Key_f9)) { gVals[8]^=1; printf("Node tasks visualization 8 %s\n",gVals[8]?"on":"off"); }
		if(window.KeyDown(Key_f10)) { gVals[9]^=1; printf("Antialiasing 4x %s\n",gVals[9]?"on":"off"); }
		if(window.KeyDown(Key_f1)) { gVals[0]^=1; printf("Traversing from 8x8: %s\n", gVals[0]?"on" : "off"); }

		static float animPos = 0;
		if(window.Key(Key_space)) animPos+=0.025f;

		double time = GetTime(); 
		Camera camera = orbiting?(Camera)ocam : (Camera)cam;

		dicom.Blit(image, slice);

		window.RenderImage(image, true);

		time = GetTime() - time;

		double fps = double(unsigned(frmCounter.FPS() * 100)) * 0.01;

		font.BeginDrawing(resx,resy);
		font.SetSize(Vec2f(30, 20));
		font.PrintAt(Vec2f(5, 25), "FPS: ", fps, " Slice: ", slice);

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
