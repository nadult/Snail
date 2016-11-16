#include <iostream>
#include <fstream>
#include <fwk_gfx.h>
#include <fwk_input.h>
#include "camera.h"

#include "formats/loader.h"
#include "base_scene.h"

#include "mipmap_texture.h"
#include "render.h"
#include "shading/material.h"

#include "bvh/tree.h"
#include "dbvh/tree.h"

#include "scene.h"
#include "frame_counter.h"

#include "render_opengl.h"
#include "photons.h"

using std::cout;
using std::endl;
using fwk::getTime;
using fwk::Loader;
using fwk::Saver;

namespace InputKey = fwk::InputKey;
using fwk::InputButton;

static void PrintHelp() {
	printf("Synopsis:    rtracer model_file [options]\nOptions:\n"
			"\t-res x y     - set rendering resolution [1024 1024]\n"
			"\t-fullscreen\n"
			"\t-rebuild     - rebuild BVH tree\n"
			"\t-fastRebuild - rebuild BVH tree quickly (slower to traverse)\n"
			"\t-flipNormals - flip normals of triangles when rebuilding\n"
			"\t-swapYZ      - swap Y, Z axes when rebuilding\n"
			"\t-noSah       - don't use SAH when rebuilding\n"
			"\t-instances n	- draw N istances of given model\n"
			"\n\n"
			"Interactive control:\n"
			"\tA,W,S,D R,F         - move the camera\n"
			"\tmouse + left button - rotate camera\n"
			"\tmouse wheel         - zoom in/out camera (works only in orbit mode)\n"
			"\tright mouse button  - flip mouse axes (works only in orbit mode)\n"
			"\tshift               - faster movement / zooming\n"
			"\tk   - save image to out/output.tga\n"
			"\to   - toggle camera mode (FPS / orbiting)\n"
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

static void MoveCamera(OrbitingCamera &cam, fwk::GfxDevice &device, float speed) {
	auto input = device.inputState();

	if(input.isKeyPressed(InputKey::lshift)) speed *= 5.f;
	if(input.isMouseButtonPressed(InputButton::left) || input.isMouseButtonPressed(InputButton::right)) {
		float flip = input.isMouseButtonPressed(InputButton::right)? -1 : 1;

		device.grabMouse(true);
		float dx = input.mouseMove().x * flip;
		float dy = input.mouseMove().y * flip;

		if(dx) cam.Rotate(-dx * 0.01);
		if(dy) cam.RotateY(dy * 0.01);
	}
	else {
		device.grabMouse(false);
	}
		
	if(input.isKeyPressed(InputKey::left)) cam.Rotate(0.01);
	if(input.isKeyPressed(InputKey::right)) cam.Rotate(-0.01);
	
	float zoom = 0;
	zoom = input.mouseWheelMove() * 4;
	if(input.isKeyPressed('R')) zoom = 1;
	if(input.isKeyPressed('F')) zoom = -1;

	if(zoom)
		cam.Zoom(zoom * speed);

	Vec3f move(0, 0, 0);
	Camera tcam = (Camera)cam;

	if(input.isKeyPressed('w')) move += tcam.front;
	if(input.isKeyPressed('s')) move -= tcam.front;
	if(input.isKeyPressed('a')) move -= tcam.right;
	if(input.isKeyPressed('d')) move += tcam.right;

	cam.target += move * speed;
	cam.pos += move * speed;
}

static void MoveCamera(FPSCamera &cam, fwk::GfxDevice &device, float speed) {
	auto input = device.inputState();

	if(input.isKeyPressed(InputKey::lshift)) speed *= 5.f;
	if(input.isMouseButtonPressed(InputButton::left)) {
		device.grabMouse(true);
		float dx = input.mouseMove().x;
		float dy = input.mouseMove().y;

		if(dx) cam.Rotate(-dx * 0.01);
		if(dy) cam.RotateY(dy * 0.01);
	}
	else {
		device.grabMouse(false);
	}

	Vec3f move(0, 0, 0);
	Camera tcam = (Camera)cam;

	if(input.isKeyPressed('w')) move += tcam.front;
	if(input.isKeyPressed('s')) move -= tcam.front;
	if(input.isKeyPressed('a')) move -= tcam.right;
	if(input.isKeyPressed('d')) move += tcam.right;
	if(input.isKeyPressed('r')) move += tcam.up;
	if(input.isKeyPressed('f')) move -= tcam.up;

	cam.Move(move * speed);
}

static DBVH MakeDBVH(const BVH *bvh, int nInstances) {
	srand(0);
	static float anim = 0; anim += 0.02f;

	float scale = Length(bvh->GetBBox().Size()) * 0.015;

	vector<ObjectInstance> instances;
	for(int n = 0; n < nInstances; n++) {
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

class RayTracer {
public:
	struct Config {
		int resx = 1024, resy = 1024;
		bool fullscreen = false;
		int threads = 2;
		string sceneName = "sponza.obj";

		Options options;
		bool flipNormals = false, swapYZ = false;
		bool rebuild = false;
		string texPath = "";
		int buildFlags = BVH::useSah;

		int nInstances = 1;
	} config;

	RayTracer(Scene<BVH> &scene, const vector<shading::MaterialDesc> &matDescs, Config config)
		:m_scene(scene), matDescs(matDescs), m_config(config),
		m_font(getFont("liberation_24")), m_image(config.resx, config.resy, fwk::TextureFormatId::rgb) {
	
		sceneScale = Length(Vec3f(scene.geometry.GetBBox().Size()));
		sceneCenter = scene.geometry.GetBBox().Center();

		try {
			Loader("scenes/cameras.dat") >> m_camConfigs;
		} catch(...) {
			m_camConfigs = {};
		}

		if(!m_camConfigs.GetConfig(string(config.sceneName), cam))
			cam.SetPos(sceneCenter);
		{
			//scene.geometry.Construct(builder.ExtractElements());
			Vec3f size = scene.geometry.GetBBox().Size();
			speed = (size.x + size.y + size.z) * 0.0025f;
		}

	}

	bool processInput(fwk::GfxDevice &device) {
		auto input = device.inputState();

		frmCounter.NextFrame();

		if(input.isKeyUp(InputKey::esc))
			return false;
		if(input.isKeyDown('k')) {
			fwk::mkdirRecursive("out");
			Saver("out/output.tga") << fwk::Texture(m_image);
		}

		if(input.isKeyDown('c')) {
			if(orbiting)
				ocam.Reset(sceneCenter, sceneScale);
			else
				cam.SetPos(sceneCenter);
			cam.ang = 0;
			cam.pitch = 0;
		}
		if(input.isKeyDown('o')) {
			if(orbiting) cam.SetPos(ocam.pos);
			else ocam.Reset(cam.pos, -sceneScale);
			orbiting ^= 1;
		}
		if(input.isKeyDown('p')) {
			m_camConfigs.AddConfig(string(m_config.sceneName),cam);
			fwk::mkdirRecursive("scenes");
			Saver("scenes/cameras.dat") << m_camConfigs;
			cam.Print();
		}
		if(input.isKeyDown('l')) {
			printf("Lights %s\n",lightsEnabled?"disabled":"enabled");
			lightsEnabled^=1;
		}
		if(input.isKeyDown('j')) {
			Vec3f colors[] = {
				Vec3f(1.0, 1.0, 1.0),
				Vec3f(0.2, 0.5, 1.0),
				Vec3f(1.0, 0.5, 0.2),
				Vec3f(0.7, 1.0, 0.0),
			};

			Vec3f pos = (orbiting?(Camera)ocam : (Camera)cam).pos;
			Light newLight(pos, colors[rand() % countof(colors)], sceneScale);
			lights.emplace_back(newLight);
		}

		if(input.isKeyDown('h')) {
			oglRendering ^= 1;
			if(oglRendering && !oglRenderer) {
				try {
					oglRenderer = std::make_unique<OGLRenderer>(m_scene);
				}
				catch(const Exception &ex) {
					std::cout << ex.what();
					oglRendering = false;
				}
			}
		}

		float tspeed = speed * lastFrameTime * 20.0f;
		lastFrameTime = getTime() - lastTime;
		lastTime = getTime();

		if(orbiting)
			MoveCamera(ocam, device, tspeed);
		else
			MoveCamera(cam, device, tspeed);

		for(int n = 1; n <= 8; n++)
			if(input.isKeyDown('0' + n)) {
				m_config.threads = n;
				printf("Threads: %d\n", m_config.threads);
			}

		if(input.isKeyDown(InputKey::f1)) { gVals[0]^=1; printf("Val 0 %s\n", gVals[0]?"on" : "off"); }
		if(input.isKeyDown(InputKey::f2)) { gVals[1]^=1; printf("Val 1 %s\n", gVals[1]?"on" : "off"); }
		if(input.isKeyDown(InputKey::f3)) { gVals[2]^=1; printf("Val 2 %s\n", gVals[2]?"on" : "off"); }
		if(input.isKeyDown(InputKey::f6)) { gVals[5]^=1; printf("Scene complexity visualization %s\n",gVals[5]?"on":"off"); }
		if(input.isKeyDown(InputKey::f7)) { gVals[6]^=1; printf("Advanced shading %s\n",gVals[6]?"on":"off"); }
		if(input.isKeyDown(InputKey::f8)) { gVals[7]^=1; printf("Reflections 7 %s\n",gVals[7]?"on":"off"); }
		if(input.isKeyDown(InputKey::f9)) { gVals[8]^=1; printf("Node tasks visualization 8 %s\n",gVals[8]?"on":"off"); }
		if(input.isKeyDown(InputKey::f10)) { gVals[9]^=1; printf("Antialiasing 4x %s\n",gVals[9]?"on":"off"); }
		if(input.isKeyDown(InputKey::f1)) { gVals[0]^=1; printf("Traversing from 8x8: %s\n", gVals[0]?"on" : "off"); }

		static float animPos = 0;
		if(input.isKeyPressed(InputKey::space)) animPos+=0.025f;


		return true;
	}

	void drawImage(fwk::Renderer2D &out, const MipmapTexture &image) const {
		auto dtexture = fwk::make_shared<fwk::DTexture>(fwk::Texture(image));
		fwk::FColor colors[4] = {fwk::ColorId::white, fwk::ColorId::white, fwk::ColorId::white,
								 fwk::ColorId::white};
		fwk::int2 size(image.Width(), image.Height());
		out.addFilledRect(fwk::FRect(fwk::float2(size)), fwk::FRect(0, 0, 1, -1), colors, dtexture);
	}

	void draw(fwk::GfxDevice &device) {
		double buildTime = getTime();
		Scene<DBVH> dscene;
		if(m_config.nInstances > 1) {
			dscene.geometry = MakeDBVH(&m_scene.geometry, m_config.nInstances);
			dscene.materials = m_scene.materials;
		}
		buildTime = getTime() - buildTime;

		vector<Light> tLights = lightsEnabled?lights:vector<Light>();
		//	for(int n=0;n<tLights.size();n++)
		//		tLights[n].pos += Vec3f(sin(animPos+n*n),cos(animPos+n*n),
		//			sin(animPos-n*n)*cos(animPos+n*n)) * tspeed;
		m_scene.lights = tLights;
		dscene.lights = tLights;

		fwk::Renderer2D renderer(fwk::IRect(0, 0, m_config.resx, m_config.resy));

		double time = getTime(); 
		TreeStats stats; {
			Camera camera = orbiting?(Camera)ocam : (Camera)cam;

			if(oglRendering) {
				oglRenderer->BeginDrawing(camera, 60.0f, float(m_config.resx) / float(m_config.resy), 1);
				oglRenderer->Draw();
				oglRenderer->FinishDrawing();
			}
			else {
				stats = m_config.nInstances > 1? Render(dscene, camera, m_image, m_config.options, m_config.threads) :
										Render( m_scene, camera, m_image, m_config.options, m_config.threads);
				bool updated = 0;
				drawImage(renderer, m_image);
			}
		}

		time = getTime() - time;

		double fps = double(unsigned(frmCounter.FPS() * 100)) * 0.01;
		double mrays = double(unsigned(frmCounter.FPS() * stats.GetRays() * 0.0001)) * 0.01;

		auto font = getFont("liberation_24");
		char text[256];
		auto style = fwk::FontStyle{fwk::ColorId::white, fwk::ColorId::black};

		if(oglRendering) {
			font.draw(renderer, {5.0f, 25.0f}, style, fwk::format("FPS: %f (opengl rendering)", fps));
		}
		else {
			font.draw(renderer, {5.0f, 5.0f}, style, stats.GenInfo(m_config.resx, m_config.resy, time * 1000.0, buildTime * 1000.0));
			font.draw(renderer, {5.0f, 25.0f}, style, fwk::format("FPS: %.2f %s%.2f", fps, " MRays/sec:", mrays));
			if(lightsEnabled && lights.size())
				font.draw(renderer, {5.0f, 65.0f}, style, fwk::format("Lights: %d", lightsEnabled?(int)lights.size() : 0));
	//		snprintf(text, sizeof(text), "prim:", gVals[1], ' ', gVals[2], ' ', gVals[3],
	//			" sh:", gVals[4], " refl:", gVals[5], ' ', gVals[6], " smart:", gVals[7]);
	//		font.PrintAt(Vec2f(5, 85), text);
		}

		device.clearColor(fwk::ColorId::black);
		renderer.render();
	}

	Scene<BVH> &m_scene; // TODO: const?
	const vector<shading::MaterialDesc> &matDescs;
	Config m_config;
	CameraConfigs m_camConfigs;
	fwk::Font m_font;
	MipmapTexture m_image;

	float sceneScale = 1.0f;
	Vec3f sceneCenter;
		
	FPSCamera cam;
	OrbitingCamera ocam;

	vector<Light> lights;// = GenLights();
	bool lightsEnabled = 1;
	bool staticEnabled = 0, orbiting = 0;
	float speed;

	std::unique_ptr<OGLRenderer> oglRenderer;
	bool oglRendering = 0;

	FrameCounter frmCounter;
	float lastFrameTime = 0.0f;
	double lastTime = getTime();
};

static RayTracer *s_rtracer = nullptr;

bool mainLoop(fwk::GfxDevice &device) {
	DASSERT(s_rtracer);
	if(!s_rtracer->processInput(device))
		return false;
	s_rtracer->draw(device);
	return true;
}

static int tmain(int argc, char **argv) {
	float nan = 0.0f / 0.0f;
	float not_nan = 3.0f;

//	_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
	ASSERT(IsNan(Vec3f(nan, nan, nan)));
	ASSERT(isnan(nan));
	ASSERT(!isnan(not_nan));
	Vec3q nan3;
	Convert(Vec3f(nan, nan, nan), Vec3f(nan, nan, nan), Vec3f(nan, nan, nan), Vec3f(nan, nan, nan), nan3);
	ASSERT(IsNan(nan3));

	printf("Snail v0.20 by nadult\n");
	if(argc>=2&&string("--help")==argv[1]) {
		PrintHelp();
		return 0;
	}
	else printf("type './rtracer --help' to get some help\n");

	RayTracer::Config conf;

	for(int n = 1; n < argc; n++) {
		if(string("-res") == argv[n] && n < argc-2) {
			 conf.resx = atoi(argv[n+1]);
			 conf.resy = atoi(argv[n+2]); n += 2;
		 }
		else if(string("-fastRebuild") == argv[n]) { conf.rebuild = 1; conf.buildFlags |= BVH::fastBuild; }
		else if(string("-rebuild") == argv[n]) { conf.rebuild = 1; }
		else if(string("-threads") == argv[n] && n < argc - 1) { conf.threads = atoi(argv[n + 1]); n += 1; }
		else if(string("-fullscreen") == argv[n]) conf.fullscreen = 1;
		else if(string("-flipNormals") == argv[n]) conf.flipNormals = 1;
		else if(string("-swapYZ") == argv[n]) conf.swapYZ = 1;
		else if(string("-noSah") == argv[n]) conf.buildFlags &= ~BVH::useSah;
		else if(string("-noShadingData") == argv[n]) conf.buildFlags |= BVH::noShadingData;
		else if(string("-instances") == argv[n]) { conf.nInstances = atoi(argv[n + 1]); n += 1; }
		else {
			if(argv[n][0] == '-') {
				printf("Unknown option: %s\n",argv[n]);
				exit(0);
			}
			else conf.sceneName = argv[n];
		}
	}
	
	if(conf.texPath == "") {
		conf.texPath = "scenes/" + conf.sceneName;
		auto pos = conf.texPath.rfind('/');
		if(pos == string::npos) conf.texPath = "";
		else conf.texPath.resize(pos + 1);
	}

	Scene<BVH> scene;
	if(!conf.rebuild) {
		try {
			Loader(string("dump/") + conf.sceneName) >> scene.geometry;
		}
		catch(...) { conf.rebuild = 1; }
	}
	if(conf.rebuild) {
		printf("Loading...\n");
		double loadingTime = getTime();
		BaseScene baseScene; {
			string fileName = string("scenes/") + conf.sceneName;
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

					for(const auto &obj : tmp.objects) {
						if(obj.tris.size() < 800000) {
							baseScene.objects.push_back(obj);
							ntris += baseScene.objects.back().tris.size();
						}

					}
					if(ntris > 4000000)
						break;
				}
			}
			else THROW("Unrecognized format: %s", fileName.c_str());

			int tris = 0;
			for(int n = 0; n < baseScene.objects.size(); n++)
				tris += baseScene.objects[n].tris.size();
			printf("Tris: %d\n",tris);

			// We're flipping normals by default
			if(!conf.flipNormals)
				baseScene.FlipNormals();
			if(conf.swapYZ) baseScene.SwapYZ();
		//	for(int n = 0; n < baseScene.objects.size(); n++)
		//		baseScene.objects[n].Repair();
			baseScene.GenNormals();
		//	baseScene.Optimize();
		}
		loadingTime = getTime() - loadingTime;
		
		double buildTime = getTime();
		scene.geometry.Construct(baseScene, conf.buildFlags);
		fwk::mkdirRecursive("dump");
		Saver(string("dump/") + conf.sceneName) << scene.geometry;
		buildTime = getTime() - buildTime;
		std::cout << "Loading time: " << loadingTime << "  Build time: " << buildTime << '\n';
	}
	scene.geometry.PrintInfo();

	vector<shading::MaterialDesc> matDescs;
	if(conf.sceneName.substr(conf.sceneName.size() - 4) == ".obj") {
		string mtlFile = "scenes/" + conf.sceneName.substr(0, conf.sceneName.size() - 3) + "mtl";
		matDescs = shading::LoadMaterialDescs(mtlFile);
	}
		
	scene.texDict = LoadTextures(matDescs, conf.texPath);
	scene.matDict = shading::MakeMaterials(matDescs, scene.texDict);
		
	scene.UpdateMaterials();
	scene.geometry.UpdateMaterialIds(scene.GetMatIdMap());


	fwk::GfxDevice device;
	uint gfx_flags = conf.fullscreen? fwk::GfxDevice::flag_fullscreen_desktop : 0;
	device.createWindow("Snail realtime raytracer", {conf.resx, conf.resy}, gfx_flags);
	conf.resx = device.windowSize().x;
	conf.resy = device.windowSize().y;

	RayTracer rtracer(scene, matDescs, conf);
	s_rtracer = &rtracer;
	device.runMainLoop(mainLoop);

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
