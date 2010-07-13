#include "pch.h"
#include <iostream>
#include "camera.h"

#include "gl_window.h"
#include "formats/loader.h"
#include "base_scene.h"
#include "bvh/tree.h"

#include "scene.h"
#include "mesh.h"
#include "frame_counter.h"
#include "render.h"

#include "font.h"
#include "frame_counter.h"

#include <boost/asio.hpp>
#include "quicklz/quicklz.h"

using boost::asio::ip::tcp;

int gVals[16]={0,};
double gdVals[16]={0,};

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

template <class Scene>
static void SetMaterials(Scene &scene,const BaseScene &base,string texPath) {
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

static vector<Light> GenLights(float scale = 1.0f, float power = 1000.0f) {
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

void ReadSocket(tcp::socket &socket, void *ptr, int size) {
	int bytes = 0;
	while(bytes < size) {
		int len = socket.read_some(boost::asio::buffer(((char*)ptr) + bytes, size - bytes));
		bytes += len;
	}
}

void WriteSocket(tcp::socket &socket, const void *ptr, int size) {
	boost::asio::write(socket, boost::asio::buffer(ptr, size));
}

void SendFrameRequest(tcp::socket &socket, const Camera &cam, const vector<Light> &lights,
					int threads, bool finish) {
	WriteSocket(socket, &finish, sizeof(finish));
	WriteSocket(socket, &cam, sizeof(cam));
	int size = lights.size(); WriteSocket(socket, &size, sizeof(size));
	WriteSocket(socket, &lights[0], size * sizeof(Light));
	WriteSocket(socket, gVals, sizeof(gVals));
	WriteSocket(socket, &threads, sizeof(threads));
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

//	Mesh mesh;
//	MeshAnim meshAnim;
//	mesh.Load("scenes/doom3/imp/imp.md5mesh");
//	meshAnim.Load("scenes/doom3/imp/walk1.md5anim");

//	StaticTree meshTree;

	int resx=1024, resy=1024;
	bool fullscreen = 0;
	int threads = 2;
	const char *modelFile = "foot.obj";
	const char *host = "blader";

	Options options;
	bool flipNormals = 1;
	bool rebuild = 0;
	string texPath = "scenes/";

	for(int n=1;n<argc;n++) {
			 if(string("-res")==argv[n]&&n<argc-2) { resx=atoi(argv[n+1]); resy=atoi(argv[n+2]); n+=2; }
		else if(string("-rebuild") == argv[n]) { rebuild = 1; }
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

	Scene<StaticTree> staticScene;
	if(!rebuild) {
		try { Loader(string("dump/") + modelFile) & staticScene.geometry; }
		catch(...) { rebuild = 1; }
	}
	if(rebuild) {
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
		
		staticScene.geometry.Construct(baseScene.ToCompactTris());
	//	staticScene.geometry.Construct(baseScene.ToTriangleVector());
		Saver(string("dump/") + modelFile) & staticScene.geometry;
		buildTime = GetTime() - buildTime;
		std::cout << "Build time: " << buildTime << '\n';
	}
	staticScene.geometry.PrintInfo();
	staticScene.materials.push_back(shading::NewMaterial(""));
	staticScene.Update();

	float sceneScale; {
		Vec3f size = staticScene.geometry.GetBBox().Size();
		sceneScale = Length(size);
	}

	GLWindow window(resx, resy, fullscreen);
	Font font;

	gfxlib::Texture image(resx, resy, gfxlib::TI_A8B8G8R8);

	double minTime = 1.0f / 0.0f, maxTime = 0.0f;
	vector<Light> lights;// = GenLights();
			
	Camera cam;
	if(!camConfigs.GetConfig(string(modelFile),cam))
		cam.SetPos(staticScene.geometry.GetBBox().Center());


	bool lightsEnabled=1;
	bool staticEnabled=0;
	float speed; {
	//	scene.geometry.Construct(builder.ExtractElements());
		Vec3p size = staticScene.geometry.GetBBox().Size();
		speed = (size.x + size.y + size.z) * 0.0025f;
	}
	
//	Vec3f colors[4]={Vec3f(1,1,1),Vec3f(0.2,0.5,1),Vec3f(0.5,1,0.2),Vec3f(0.7,1.0,0.0)};
//	lights.push_back(Light(cam.Pos(), colors[rand()&3], 800.0f * 0.001f * sceneScale));
//	lights.push_back(Light(cam.Pos(), colors[rand()&3], 800.0f * 0.001f * sceneScale));
//	lights.push_back(Light(cam.Pos(), colors[rand()&3], 800.0f * 0.001f * sceneScale));
//	lights.push_back(Light(cam.Pos(), colors[rand()&3], 800.0f * 0.001f * sceneScale));

	FrameCounter frmCounter;
		
	boost::asio::io_service ioService;
	tcp::resolver resolver(ioService);
	tcp::resolver::query query(host, "20000");
	tcp::resolver::iterator endpointIterator = resolver.resolve(query);
	tcp::resolver::iterator end;

	tcp::socket socket(ioService);
	boost::system::error_code error = boost::asio::error::host_not_found;
	while(error && endpointIterator != end) {
		socket.close();
		socket.connect(*endpointIterator++, error);
	}
	if(error)
		ThrowException("Error while connecting to server: ", error);

	{
		char model[256]; snprintf(model, 256, "%s", modelFile);
		
		WriteSocket(socket, &resx, 4);
		WriteSocket(socket, &resy, 4);
		WriteSocket(socket, model, sizeof(model));
	}

	bool finish = 0;
	SendFrameRequest(socket, cam, lights, threads, finish);

	while(!finish) {
		if(!window.PollEvents() || window.KeyUp(Key_esc)) finish = 1;
		frmCounter.NextFrame();
		
		if(window.KeyDown('K'))
			Saver("out/output.dds") & image;
		if(window.KeyDown('I'))
			options.rdtscShader ^= 1;
		if(window.KeyDown('C'))
			cam.SetPos(staticScene.geometry.GetBBox().Center());
		if(window.KeyDown('P')) {
			camConfigs.AddConfig(string(modelFile),cam);
			Saver("scenes/cameras.dat") & camConfigs;
			cam.Print();
		}
		if(window.KeyDown('L')) {
			printf("Lights %s\n", lightsEnabled? "disabled" : "enabled");
			lightsEnabled ^= 1;
		}
		if(window.KeyDown('J')) {
			Vec3f colors[4] = {
				Vec3f(1,1,1), Vec3f(0.2,0.5,1),Vec3f(0.5,1,0.2),Vec3f(0.7,1.0,0.0) };

			lights.push_back(Light(cam.Pos(), colors[rand()&3], 1500.0f * 0.001f * sceneScale));
		}

		MoveCamera(cam, window, speed);

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
		if(window.KeyDown(Key_f9)) { gVals[8]^=1; printf("Val 8 %s\n",gVals[8]?"on":"off"); }
		if(window.KeyDown(Key_f10)) { gVals[9]^=1; printf("Val 9 %s\n",gVals[9]?"on":"off"); }


		static float animPos = 0;
		if(window.Key(Key_space)) animPos += 0.025f;

		vector<Light> tLights = lightsEnabled?lights:vector<Light>();
			for(int n=0;n<tLights.size();n++)
				tLights[n].pos += Vec3f(sin(animPos+n*n),cos(animPos+n*n),
					sin(animPos-n*n)*cos(animPos+n*n))*speed;
		staticScene.lights /*= scene.lights*/ = tLights;
		staticScene.Update();
	//	scene.Update();
		double time = GetTime();
		int w = image.Width(), h = image.Height();
		vector<char> compressed(w * h * 4 + 400), decompressed(w * h * 4);

		int nBytes = 0;
		TreeStats<1> stats; {

			int nPixels = w * h, received = 0;
			SendFrameRequest(socket, cam, lights, threads, finish);

			static bool first = 1;
			if(!first) {
				char dummy[1460];
				ReadSocket(socket, dummy, sizeof(dummy));
			} else first = 0;

			while(received < nPixels) {
				struct { int x, y, w, h, size; } info;
				ReadSocket(socket, &info, sizeof(info));
				ReadSocket(socket, &compressed[0], info.size);
				nBytes += info.size;

				InputAssert(qlz_size_decompressed(&compressed[0]) == info.w * info.h * 4);

				char scratch[QLZ_SCRATCH_DECOMPRESS];
				qlz_decompress(&compressed[0], &decompressed[0], scratch);

				for(int y = 0; y < info.h; y++) {
					const char *csrc = &decompressed[0] + y * info.w * 4;
					char *cdst = (char*)image.DataPointer() + (y + info.y) * image.Pitch() +
									info.x * 4;
					std::copy(csrc, csrc + info.w * 4, cdst);
				}
				received += info.w * info.h;
			}
		
			ReadSocket(socket, &stats, sizeof(stats));	
		}

		time = GetTime() - time;

		window.RenderImage(image);

		double fps = double(unsigned(frmCounter.FPS() * 100)) * 0.01;
		double mrays = double(unsigned(frmCounter.FPS() * stats.GetRays() * 0.0001)) * 0.01;

		font.BeginDrawing(resx,resy);
		font.SetSize(Vec2f(30, 20));
			font.PrintAt(Vec2f(5,  5), stats.GenInfo(resx, resy, time * 1000.0, 0));
			font.PrintAt(Vec2f(5, 25), "FPS: ", fps, " MRays/sec:", mrays, " KBytes/frame:", nBytes / 1024);
			if(lightsEnabled && lights.size())
				font.PrintAt(Vec2f(5, 45), "Lights: ",lightsEnabled?lights.size() : 0);
			font.PrintAt(Vec2f(5, 85), "prim:", gVals[1], ' ', gVals[2], ' ', gVals[3],
					" sh:", gVals[4], " refl:", gVals[5], ' ', gVals[6], " smart:", gVals[7], ' ',
					gdVals[0] / sceneScale, ' ', gdVals[1] / sceneScale);
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
