#include "pch.h"
#include <iostream>
#include "camera.h"

#include "formats/loader.h"
#include "base_scene.h"

#include "render.h"
#include "shading/material.h"

#include "bvh/tree.h"

#include "scene.h"
#include "mesh.h"

#include <mpi.h>

#include <boost/asio.hpp>

using boost::asio::ip::tcp;

using std::cout;
using std::endl;

int gVals[16]={0, };

typedef BVH StaticTree;

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

void ReadSocket(tcp::socket &socket, void *ptr, int size) {
	int bytes = 0;
	while(bytes < size) {
		int len = socket.read_some(boost::asio::buffer((char*)ptr + bytes, size - bytes));
		bytes += len;
	}
}

void WriteSocket(tcp::socket &socket, void *ptr, int size) {
	boost::asio::write(socket, boost::asio::buffer((char*)ptr, size));
}

static char sendbuf[1024 * 1024 * 4];

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

	vector<CompressedPart> parts;

	while(true) {
		if(rank == 0) {
			boost::asio::io_service ioService;
			tcp::acceptor acceptor(ioService, tcp::endpoint(tcp::v4(), 20000));
			tcp::socket socket(ioService);
			acceptor.accept(socket);

			int resx, resy, threads = 2;
			char sceneName[256];
			ReadSocket(socket, &resx, sizeof(resx));
			ReadSocket(socket, &resy, sizeof(resy));
			ReadSocket(socket, sceneName, sizeof(sceneName));
			
			for(int r = 1; r < numNodes; r++) {
				MPI_Send(&resx, sizeof(resx), MPI_CHAR, r, 0, MPI_COMM_WORLD);
				MPI_Send(&resy, sizeof(resy), MPI_CHAR, r, 1, MPI_COMM_WORLD);
				MPI_Send(&sceneName, sizeof(sceneName), MPI_CHAR, r, 2, MPI_COMM_WORLD);
			}

			Scene<StaticTree> scene;
			//TODO: try catch
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
				ReadSocket(socket, &finish, sizeof(finish));
				for(int r = 1; r < numNodes; r++)
					MPI_Send(&finish, sizeof(finish), MPI_CHAR, r, 0, MPI_COMM_WORLD);
				if(finish) {
					printf("Disconnected\n");
					break;
				}

				int nLights;

				ReadSocket(socket, &cam, sizeof(cam));
				ReadSocket(socket, &nLights, sizeof(nLights));
				scene.lights.resize(nLights);
				ReadSocket(socket, &scene.lights[0], nLights * sizeof(Light));
				ReadSocket(socket, gVals, sizeof(gVals));
				ReadSocket(socket, &threads, sizeof(threads));

				double time = GetTime();
				for(int r = 1; r < numNodes; r++) {
					MPI_Send(&cam, sizeof(cam), MPI_CHAR, r, 1, MPI_COMM_WORLD);
					MPI_Send(&nLights, sizeof(nLights), MPI_CHAR, r, 2, MPI_COMM_WORLD);
					MPI_Send(&scene.lights[0], sizeof(Light) * nLights, MPI_CHAR, r, 3, MPI_COMM_WORLD);
					MPI_Send(gVals, sizeof(gVals), MPI_CHAR, r, 4, MPI_COMM_WORLD);
					MPI_Send(&threads, sizeof(threads), MPI_CHAR, r, 5, MPI_COMM_WORLD);
				}

				int nPixels = w * h, received = 0;
				TreeStats<1> stats = 
					Render(scene, cam, image, rank, numNodes, lineHeight, options, threads);
				int nParts = CompressParts(image, rank, numNodes, lineHeight, threads, parts);
				for(int p = 0; p < nParts; p++) {
					received += parts[p].info.w * parts[p].info.h;
					WriteSocket(socket, &parts[p].info, sizeof(CompressedPart::Info));
					WriteSocket(socket, &parts[p].data[0], parts[p].info.size);
				}

				while(received < nPixels) {
					MPI_Status status;
					MPI_Recv(&parts[0].info, sizeof(CompressedPart::Info), MPI_CHAR, MPI_ANY_SOURCE, 0, MPI_COMM_WORLD, &status);
					received += parts[0].info.w * parts[0].info.h;
					if(parts[0].data.size() < parts[0].info.size)
						parts[0].data.resize(parts[0].info.size);
					MPI_Recv(&parts[0].data[0], parts[0].info.size, MPI_CHAR, status.MPI_SOURCE, 0, MPI_COMM_WORLD, &status);
					WriteSocket(socket, &parts[0].info, sizeof(CompressedPart::Info));
					WriteSocket(socket, &parts[0].data[0], parts[0].info.size);
				}
				for(int r = 1; r < numNodes; r++) {
					TreeStats<1> nodeStats;
					MPI_Status status;
					MPI_Recv(&nodeStats, sizeof(nodeStats), MPI_CHAR, MPI_ANY_SOURCE, 1, MPI_COMM_WORLD, &status);
					stats += nodeStats;
				}
				WriteSocket(socket, &stats, sizeof(stats));
				time = GetTime() - time;

				char dummy[1460];
				WriteSocket(socket, dummy, sizeof(dummy));

				printf("%s at %dx%d on %d nodes, each with %d threads: %.2f ms\n",
						sceneName, resx, resy, numNodes, threads, time * 1000.0);
			}

		}
		else {
			MPI_Status status;

			int resx, resy, threads = 2;
			char sceneName[256];
			MPI_Recv(&resx, sizeof(resx), MPI_CHAR, 0, 0, MPI_COMM_WORLD, &status);
			MPI_Recv(&resy, sizeof(resy), MPI_CHAR, 0, 1, MPI_COMM_WORLD, &status);
			MPI_Recv(&sceneName, sizeof(sceneName), MPI_CHAR, 0, 2, MPI_COMM_WORLD, &status);

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
				MPI_Recv(&finish, sizeof(finish), MPI_CHAR, 0, 0, MPI_COMM_WORLD, &status);
				if(finish) break;
				int nLights;

				MPI_Recv(&cam, sizeof(cam), MPI_CHAR, 0, 1, MPI_COMM_WORLD, &status);
				MPI_Recv(&nLights, sizeof(nLights), MPI_CHAR, 0, 2, MPI_COMM_WORLD, &status);
				scene.lights.resize(nLights);
				MPI_Recv(&scene.lights[0], sizeof(Light) * nLights, MPI_CHAR, 0, 3, MPI_COMM_WORLD, &status);
				MPI_Recv(gVals, sizeof(gVals), MPI_CHAR, 0, 4, MPI_COMM_WORLD, &status);
				MPI_Recv(&threads, sizeof(threads), MPI_CHAR, 0, 5, MPI_COMM_WORLD, &status);
		
				TreeStats<1> stats = 
					Render(scene, cam, image, rank, numNodes, lineHeight, options, threads);
				int nParts = CompressParts(image, rank, numNodes, lineHeight, threads, parts);

				for(int p = 0; p < parts.size(); p++) {
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
