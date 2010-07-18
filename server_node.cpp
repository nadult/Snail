#include "pch.h"
#include <iostream>
#include "camera.h"

#include "formats/loader.h"
#include "base_scene.h"

#include "render.h"
#include "shading/material.h"

#include "bvh/tree.h"
#include "dbvh/tree.h"

#include "scene.h"
#include "mesh.h"

#include <mpi.h>
#include "comm.h"

using comm::Socket;
using comm::MPINode;
using comm::MPIAnyNode;
using comm::Pod;

#include "compression.h"

using std::cout;
using std::endl;

typedef BVH StaticTree;

static char sendbuf[1024 * 1024 * 4];

static const DBVH MakeDBVH(BVH *bvh) {
	srand(0);
	static float anim = 0; anim += 0.02f;

	float scale = Length(bvh->GetBBox().Size()) * 0.05f;

	vector<ObjectInstance> instances;
/*	for(int n = 0; n < 10000; n++) {
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
	} */
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


static int server_main(int argc, char **argv) {
	int rank, numNodes, maxNodes;
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);
	MPI_Comm_size(MPI_COMM_WORLD, &numNodes);
	maxNodes = numNodes;
	printf("Snail v 0.20 node %d / %d\n", rank, numNodes);

	MPI_Buffer_attach(sendbuf, sizeof(sendbuf));

	enum {
		lineHeight = 16,
	};

	vector<CompressedPart> parts(8);

	while(true) {
		if(rank == 0) {
			Socket socket;
			socket.Accept(20000);

			int resx, resy, threads = 2;
			char sceneName[256];
			socket >> Pod(resx) >> Pod(resy) >> Pod(sceneName);
			
			for(int r = 1; r < numNodes; r++)
				MPINode(r, 0) << Pod(resx) << Pod(resy) << Pod(sceneName);

			Scene<StaticTree> scene;
			//TODO: try catch
			Loader(string("dump/") + sceneName) & scene.geometry;
			scene.geometry.UpdateCache();
			scene.materials.push_back(shading::NewMaterial(""));
			scene.Update();

			Vec3f sceneCenter = scene.geometry.GetBBox().Center();
			Vec3f sceneSize = scene.geometry.GetBBox().Size();
			socket << Pod(sceneCenter) << Pod(sceneSize);

			gfxlib::Texture image(resx, resy, gfxlib::TI_A8B8G8R8);
			vector<Light> lights;
			Camera cam;
			Options options;
	
			int w = image.Width(), h = image.Height();

			for(;;) {
				bool finish;
				socket >> Pod(finish);
				for(int r = 1; r < numNodes; r++)
					MPINode(r, 0) << Pod(finish);
				if(finish) {
					printf("Disconnected\n");
					break;
				}

				int nLights;

				socket >> Pod(cam) >> Pod(nLights);
				scene.lights.resize(nLights);
				socket >> comm::Data(&scene.lights[0], nLights * sizeof(Light));
				socket >> Pod(gVals) >> Pod(threads);

				double time = GetTime();
				for(int r = 1; r < numNodes; r++) {
					MPINode(r, 1) << comm::Pod(cam) << comm::Pod(nLights)
						<< comm::Data(&scene.lights[0], sizeof(Light) * nLights)
						<< comm::Data(gVals, sizeof(gVals)) << comm::Pod(threads);
				}

				double buildTime = GetTime();
				Scene<DBVH> dscene;
				dscene.geometry = MakeDBVH(&scene.geometry);
				dscene.materials.push_back(shading::NewMaterial(""));
				dscene.lights = scene.lights;
				dscene.Update();
				buildTime = GetTime() - buildTime;

				int nPixels = w * h, received = 0;
				TreeStats stats = 
					Render(dscene, cam, image, rank, numNodes, lineHeight, options, threads);
				int nParts = CompressParts(image, rank, numNodes, lineHeight, parts, threads);

				for(int p = 0; p < nParts; p++) {
					received += parts[p].info.w * parts[p].info.h;
					socket	<< Pod(parts[p].info)
							<< comm::Data(&parts[p].data[0], parts[p].info.size);
				}

				while(received < nPixels) {
					int source;
					MPIAnyNode(&source, 0) >> comm::Pod(parts[0].info);
					received += parts[0].info.w * parts[0].info.h;
					if(parts[0].data.size() < parts[0].info.size)
						parts[0].data.resize(parts[0].info.size);
					MPINode(source, 0) >> comm::Data(&parts[0].data[0], parts[0].info.size);

					socket	<< Pod(parts[0].info)
							<< comm::Data(&parts[0].data[0], parts[0].info.size);
				}

				for(int r = 1; r < numNodes; r++) {
					TreeStats nodeStats;
					MPIAnyNode(0, 1) >> comm::Pod(nodeStats);
					stats += nodeStats;
				}
				socket << Pod(stats) << Pod(buildTime);
				time = GetTime() - time;

				printf("%s at %dx%d on %d nodes, each with %d threads: %.2f ms\n",
						sceneName, resx, resy, numNodes, threads, time * 1000.0);
			}

		}
		else {
			int resx, resy, threads = 2;
			char sceneName[256];
			MPINode(0, 0) >> Pod(resx) >> Pod(resy) >> Pod(sceneName);

			Scene<StaticTree> scene;
			Loader(string("dump/") + sceneName) & scene.geometry;
			scene.geometry.UpdateCache();
			scene.materials.push_back(shading::NewMaterial(""));
			scene.Update();

			gfxlib::Texture image(resx, resy, gfxlib::TI_A8B8G8R8);
			vector<Light> lights;
			Camera cam;
			Options options;
			
			int w = image.Width(), h = image.Height();

			for(;;) {
				bool finish;
				MPINode(0, 0) >> Pod(finish);
				if(finish) break;
				int nLights;

				MPINode(0, 1) >> Pod(cam) >> Pod(nLights);
				scene.lights.resize(nLights);
				MPINode(0, 3) >> comm::Data(&scene.lights[0], sizeof(Light) * nLights)
				   	>> Pod(gVals) >> Pod(threads);
		
				Scene<DBVH> dscene;
				dscene.lights = scene.lights;
				dscene.geometry = MakeDBVH(&scene.geometry);
				dscene.materials.push_back(shading::NewMaterial(""));
				dscene.lights = scene.lights;
				dscene.Update();

				//TODO: rendering i kompresja razem: watek zaraz po zrenderowaniu
				//dodaje do kolejki zadan nowe zadanie kompresji?
				TreeStats stats = 
					Render(dscene, cam, image, rank, numNodes, lineHeight, options, threads);
				int nParts = CompressParts(image, rank, numNodes, lineHeight, parts, threads);

				for(int p = 0; p < nParts; p++) {
					MPI_Bsend(&parts[p].info, sizeof(CompressedPart::Info), MPI_CHAR, 0, 0, MPI_COMM_WORLD);
					MPI_Bsend(&parts[p].data[0], parts[p].info.size, MPI_CHAR, 0, 0, MPI_COMM_WORLD);
				}
				MPI_Bsend(&stats, sizeof(stats), MPI_CHAR, 0, 1, MPI_COMM_WORLD);
			}
		}
	}
	return 0;
}

int main(int argc,char **argv) {
	if(MPI_Init(0, 0) != MPI_SUCCESS) {
		std::cout << "Error initializing mpi\n";
		return 1;
	}

	try {
		int ret = server_main(argc, argv);
		MPI_Finalize();
		return ret;
	}
	catch(const std::exception &ex) {
		MPI_Finalize();
		std::cout << ex.what() << '\n';
		return 1;
	}
	catch(...) {
		MPI_Finalize();
		std::cout << "Unknown exception thrown.\n";
		throw;
	}
}
