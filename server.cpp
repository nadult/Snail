#if defined(__PPC) || defined(__PPC__)

#include "pch.h"

int server_main(int, char**) {
	ThrowException("Server (node with rank 0) can only be run on x86");
}

#else

#include "pch.h"
#include <iostream>
#include "camera.h"

#include "formats/loader.h"
#include "base_scene.h"
#include "shading/material.h"
#include "bvh/tree.h"
#include "dbvh/tree.h"

#include "scene.h"

#include <mpi.h>
#include "comm.h"
#include "tree_stats.h"
#include <algorithm>

#include "quicklz/quicklz.h"

using comm::Socket;
using comm::PSocket;
using comm::MPINode;
using comm::MPIBcast;
using comm::MPIAnyNode;
using comm::Pod;

using std::cout;
using std::endl;

typedef BVH StaticTree;

static char sendbuf[2048 * 2048 * 4];

static const DBVH MakeDBVH(BVH *bvh) {
	srand(0);
	static float anim = 0; anim += 0.02f;

	float scale = Length(bvh->GetBBox().Size()) * 0.02f;

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

static void SendString(const string &str) {
	int size = str.size();
	MPIBcast() << size << comm::Data((void*)&str[0], size);
}

static void SendTexture(const sampling::PTexture tex) {
	int width = tex->Width(), height = tex->Height();
	int size = tex->GetFormat().EvalImageSize(width, height);

	unsigned ident = tex->GetFormat().GetIdent();
	MPIBcast() << width << height << ident << size;
	MPIBcast() << comm::Data(tex->DataPointer(), size);
}

static void SendMatDescs(const vector<shading::MaterialDesc> &matDescs) {
	int size = matDescs.size();
	MPIBcast() << size;
	for(int n = 0; n < size; n++) {
		const shading::MaterialDesc &mat = matDescs[n];
		MPIBcast()	<< mat.ambient << mat.diffuse << mat.specular << mat.emissive
					<< mat.transmission << mat.illuminationModel << mat.dissolveFactor
					<< mat.specularExponent << mat.refractionIndex;
		SendString(mat.ambientMap);
		SendString(mat.diffuseMap);
		SendString(mat.specularMap);
		SendString(mat.emissiveMap);
		SendString(mat.exponentMap);
		SendString(mat.dissolveMap);
		SendString(mat.name);
	}
}

static void SendTexDict(const shading::TexDict &texDict) {
	int size = texDict.size();
	shading::TexDict::const_iterator it = texDict.begin();
	MPIBcast() << size;

	while(it != texDict.end()) {
		SendString(it->first);
		SendTexture(it->second);
		++it;
	}
}

static void CSendData(void *data, size_t size) {
	enum { blockSize = 1024 * 1024 };
	vector<char> buf(blockSize + 400);

	for(size_t off = 0; off < size;) {
		size_t tsize = Min((size_t)blockSize, size - off);
		char scratch[QLZ_SCRATCH_COMPRESS];
		int csize = qlz_compress((char*)data + off, &buf[0], tsize, scratch);
		MPIBcast() << csize << comm::Data(&buf[0], csize);
		printf("."); fflush(stdout);
		off += tsize;
	}
}

static void SendData(void *data, size_t size) {
	enum { blockSize = 1024 * 1024 };
	for(size_t off = 0; off < size;) {
		size_t tsize = Min((size_t)blockSize, size - off);
		MPIBcast() << comm::Data((char*)data + off, tsize);
		printf("."); fflush(stdout);
		off += tsize;
	}
}

static void SendBVH(BVH &tree) {
	int nNodes, nTris, nMaterials;
	nNodes = tree.nodes.size();
	nTris = tree.tris.size();
	nMaterials = tree.materials.size();

	MPIBcast() << nNodes << nTris << tree.depth << nMaterials;

	SendData(&tree.nodes[0], nNodes * sizeof(BVH::Node));
	printf(">"); fflush(stdout);

	SendData(&tree.tris[0], nTris * sizeof(Triangle));
	printf(">"); fflush(stdout);

	SendData(&tree.shTris[0], nTris * sizeof(ShTriangle));
	printf(">"); fflush(stdout);

	for(int n = 0; n < nMaterials; n++)
		SendString(tree.materials[n].name);
}

static vector<int> DivideImage(int w, int h, int blockWidth, int blockHeight) {
	vector<int> out;
	for(int y = 0; y < h; y += blockHeight)
		for(int x = 0; x < w; x += blockWidth) {
			int tw = Min(w - x, blockWidth), th = (h - y, blockHeight);

			out.push_back(x);
			out.push_back(y);
			out.push_back(tw);
			out.push_back(th);
		}
	return out;
}

int server_main(int argc, char **argv) {
	int rank, numNodes;
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);
	MPI_Comm_size(MPI_COMM_WORLD, &numNodes);
	InputAssert(rank == 0);

	printf("Snail v 0.20 server %d / %d\n", rank, numNodes);

	MPI_Buffer_attach(sendbuf, sizeof(sendbuf));

	vector<unsigned char> tdata;

	enum {
		blockWidth = 16,
		blockHeight = 64,
	};

	while(true) {
		Socket socket;
		socket.Accept(20002);
		PSocket sock(socket);

		int resx, resy, threads = 2;
		comm::LoadNewModel lm;
		sock >> lm;
		string texPath = lm.name; {
			int pos = texPath.rfind('/');
			if(pos == string::npos) texPath = "";
			else texPath.resize(pos + 1);
		}
		resx = lm.resx; resy = lm.resy;
		if(resx % blockWidth || resy % blockHeight) {
			printf("Image width / height should be a multiply of %d / %d\n",
					blockWidth, blockHeight);
			continue;
		}

		vector<int> partCoords;
		vector<vector<int> > nodeParts; if(numNodes > 1) {
			partCoords = DivideImage(resx, resy, blockWidth, blockHeight);
			nodeParts.resize(numNodes - 1);
			vector<int> order(numNodes - 1);
			for(int n = 0; n < numNodes - 1; n++)
				order[n] = n;

			vector<int> taskMap((resx / blockWidth) * (resy / blockHeight));

			for(int n = 0; n < partCoords.size() / 4; n++) {
				if(n % (numNodes - 1) == 0)
					std::random_shuffle(order.begin(), order.end());
				taskMap[n] = order[n % (numNodes - 1)];
			}

			//TODO: lepiej rozlozyc kawalki obrazka na wezly
			if(0) for(int y = 1, h = resy / blockHeight; y < h - 1; y++)
				for(int x = 1, w = resx / blockWidth; x < w - 1; x++) {
					int *cur = &taskMap[x + y * w];
					if(cur[0] == cur[-1] || cur[0] == cur[-w] || cur[0] == cur[-w-1])
						Swap(cur[0], cur[1]);
				}

			for(int n = 0; n < taskMap.size(); n++) {
				vector<int> &dst = nodeParts[taskMap[n]];
				dst.push_back(partCoords[n * 4 + 0]);
				dst.push_back(partCoords[n * 4 + 1]);
				dst.push_back(partCoords[n * 4 + 2]);
				dst.push_back(partCoords[n * 4 + 3]);
			}
		}

		Scene<StaticTree> scene;

		if(!lm.rebuild) {
			try { Loader(string("dump/") + lm.name) & scene.geometry; }
			catch(...) { lm.rebuild = 1; }
		}

		if(lm.rebuild) {
			printf("Loading...\n");
			double buildTime = GetTime();
			BaseScene baseScene; {
				string fileName = string("scenes/") + lm.name;

				try {
					if(fileName.find(".proc") != string::npos)
						baseScene.LoadDoom3Proc(fileName);
					else if(fileName.find(".obj") != string::npos)
						baseScene.LoadWavefrontObj(fileName);
					else ThrowException("Unrecognized format: ", fileName);
				}
				catch(const Exception &ex) {
					printf("Exception: %s\n", ex.what());
					continue;
				}

				int tris = 0;
				for(int n = 0; n < baseScene.objects.size(); n++)
					tris += baseScene.objects[n].tris.size();
				printf("Tris: %d\n",tris);

				if(lm.flipNormals) baseScene.FlipNormals();
				if(lm.swapYZ) baseScene.SwapYZ();
			//	for(int n = 0; n < baseScene.objects.size(); n++)
			//		baseScene.objects[n].Repair();
				baseScene.GenNormals();
			//	baseScene.Optimize();
			}
			
			scene.geometry.Construct(baseScene, lm.rebuild != 2);
		//	scene.geometry.Construct(baseScene.ToTriangleVector());
			Saver(string("dump/") + lm.name) & scene.geometry;
			buildTime = GetTime() - buildTime;
			std::cout << "Build time: " << buildTime << '\n';
		}
			
		MPIBcast() << resx << resy;

		for(int n = 1; n < numNodes; n++)
			MPINode(n, 0)	<< int(nodeParts[n - 1].size() / 4)
							<< comm::Data(&nodeParts[n - 1][0], nodeParts[n - 1].size() * 4);

		scene.geometry.PrintInfo();
		
		vector<shading::MaterialDesc> matDescs;
		if(lm.name.substr(lm.name.size() - 4) == ".obj") {
			string mtlFile = "scenes/" + lm.name.substr(0, lm.name.size() - 3) + "mtl";
			matDescs = shading::LoadMaterialDescs(mtlFile);
		}
		
		std::cout << "Sending texture data to nodes...\n";
		SendMatDescs(matDescs);
		scene.texDict = LoadTextures(matDescs, "scenes/" + texPath);
		SendTexDict(scene.texDict);
		scene.matDict = shading::MakeMaterials(matDescs, scene.texDict);

		scene.UpdateMaterials();
		scene.geometry.UpdateMaterialIds(scene.GetMatIdMap());

		std::cout << "Sending geometry data to nodes...\n";
		SendBVH(scene.geometry);

		Vec3f sceneCenter = scene.geometry.GetBBox().Center();
		Vec3f sceneSize = scene.geometry.GetBBox().Size();
		sock << sceneCenter << sceneSize;

		gfxlib::Texture image(resx, resy, gfxlib::TI_R8G8B8);
		vector<Light> lights;
		Camera cam;

		int w = image.Width(), h = image.Height();

		for(;;) {
			bool finish;

			sock >> finish;
			MPIBcast() << finish;
			if(finish) {
				printf("Disconnected\n");
				break;
			}

			int nLights;

			sock >> Pod(cam) >> nLights;
			scene.lights.resize(nLights);
			sock >> comm::Data(&scene.lights[0], nLights * sizeof(Light));
			sock >> Pod(gVals) >> threads;

			double time = GetTime();
			MPIBcast() << Pod(cam) << nLights << comm::Data(&scene.lights[0], sizeof(Light) * nLights)
						<< comm::Data(gVals, sizeof(gVals)) << threads;

			/*
			double buildTime = GetTime();
			Scene<DBVH> dscene;
			dscene.geometry = MakeDBVH(&scene.geometry);
			dscene.lights = scene.lights;
			buildTime = GetTime() - buildTime;
			*/
			double buildTime = 0.0;

			int workerNodes = numNodes - 1;
			int nFinished = 0;

			while(nFinished < workerNodes) {
				int source, size;
				MPIAnyNode(&source, 0) >> size;
				if(size == 0) {
					nFinished++;
					continue;
				}
				if(tdata.size() < Abs(size))
					tdata.resize(Abs(size));
				MPINode(source, 1) >> comm::Data(&tdata[0], Abs(size));
				sock << size << comm::Data(&tdata[0], Abs(size));
			}
			sock << 0;

			double renderTimes[32];
			TreeStats stats;

			for(int r = 1; r < numNodes; r++) {
				TreeStats nodeStats;
				int rank;
				MPIAnyNode(&rank, 1) >> Pod(nodeStats);
				double renderTime;
				MPINode(rank, 2) >> renderTime;
				if(rank < 32) renderTimes[rank - 1] = renderTime;
				stats += nodeStats;
			}
			for(int k = 0; k < sizeof(stats.timers) / sizeof(stats.timers[0]); k++)
				stats.timers[k] /= partCoords.size();

			sock << Pod(stats) << buildTime << workerNodes << Pod(renderTimes);
			time = GetTime() - time;

		//	printf("%s at %dx%d on %d nodes, each with %d threads: %.2f ms\n",
		//			lm.name.c_str(), resx, resy, workerNodes, threads, time * 1000.0);
		}
	}

	return 0;
}

#endif
