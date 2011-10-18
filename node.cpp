#include "pch.h"
#include <iostream>
#include "camera.h"

#include "render.h"
#include "shading/material.h"

#include "bvh/tree.h"
#include "dbvh/tree.h"

#include "scene.h"

#include <mpi.h>
#include <unistd.h>
#include "comm.h"
#include "quicklz/quicklz.h"

using comm::Socket;
using comm::MPINode;
using comm::MPIBcast;
using comm::MPIAnyNode;
using comm::Pod;

#include "compression.h"

using std::cout;
using std::endl;

typedef BVH StaticTree;

static char sendbuf[2048 * 2048 * 4];

static const DBVH MakeDBVH(BVH *bvh, int nInstances, float animPos) {
	srand(0);

	float scale = Length(bvh->GetBBox().Size()) * 0.02f;

	vector<ObjectInstance> instances;
	for(int n = 0; n < nInstances; n++) {
		ObjectInstance inst;
		inst.tree = bvh;
		inst.translation = (Vec3f(rand() % 1000, rand() % 1000, rand() % 1000) - Vec3f(500, 500, 500))
			* scale;
		Matrix<Vec4f> rot = Rotate(
				(rand() % 1000) * 0.002f * constant::pi + animPos,
				(rand() % 1000) * 0.002f * constant::pi + animPos * 0.2f,
				(rand() % 1000) * 0.002f * constant::pi + animPos * 0.1f);

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

#ifdef __BIG_ENDIAN
	enum { bigEndian = 1 };
#else
	enum { bigEndian = 0 };
#endif

static const string ReceiveString() {
	int size;
	MPIBcast() >> size;
	if(bigEndian) ByteSwap(&size);
	string out;
	out.resize(size);
	MPIBcast() >> comm::Data(&out[0], size);
	return out;
}

static sampling::PTexture ReceiveTexture() {
	int width, height, tident, size;
	MPIBcast() >> width >> height >> tident >> size;
	if(bigEndian) { ByteSwap(&width); ByteSwap(&height); ByteSwap(&tident); ByteSwap(&size); }
	gfxlib::TextureIdent ident = (gfxlib::TextureIdent)tident;

	sampling::PTexture newTex = new gfxlib::Texture(width, height, gfxlib::TextureFormat(ident));
	MPIBcast() >> comm::Data(newTex->DataPointer(), size);
	newTex->GenMips();
	return newTex;
}

static const shading::TexDict ReceiveTexDict() {
	int size;
	MPIBcast() >> size;

	if(bigEndian) ByteSwap(&size);
	shading::TexDict dict;
	for(int n = 0; n < size; n++) {
		string name = ReceiveString();
		dict[name] = ReceiveTexture();
	}
	return dict;
}

static const vector<shading::MaterialDesc> ReceiveMatDescs() {
	int size;
	MPIBcast() >> size;
	if(bigEndian) ByteSwap(&size);
	vector<shading::MaterialDesc> matDescs(size);

	for(int n = 0; n < size; n++) {
		shading::MaterialDesc &mat = matDescs[n];
		MPIBcast()	>> mat.ambient >> mat.diffuse >> mat.specular >> mat.emissive
					>> mat.transmission >> mat.illuminationModel >> mat.dissolveFactor
					>> mat.specularExponent >> mat.refractionIndex;
		if(bigEndian) {
			ByteSwap(&mat.ambient);
			ByteSwap(&mat.diffuse);
			ByteSwap(&mat.specular);
			ByteSwap(&mat.emissive);
			ByteSwap(&mat.transmission);
			ByteSwap(&mat.illuminationModel);
			ByteSwap(&mat.dissolveFactor);
			ByteSwap(&mat.specularExponent);
			ByteSwap(&mat.refractionIndex);
		}

		mat.ambientMap = ReceiveString();
		mat.diffuseMap = ReceiveString();
		mat.specularMap = ReceiveString();
		mat.emissiveMap = ReceiveString();
		mat.exponentMap = ReceiveString();
		mat.dissolveMap = ReceiveString();
		mat.name = ReceiveString();
	}

	return matDescs;
}

static void SwapDwords(void *data, int count) {
	u32 *dwords = (u32*)data;
	for(int n = 0; n < count; n++)
		ByteSwap(dwords + n);
}

static void ReceiveData(void *data, size_t size) {
	enum { blockSize = 1024 * 1024 };
	for(size_t off = 0; off < size;) {
		size_t tsize = Min((size_t)blockSize, size - off);
		MPIBcast() >> comm::Data((char*)data + off, tsize);
		off += tsize;
	}
}

static void CReceiveData(void *data, size_t size) {
	enum { blockSize = 1024 * 1024 };
	vector<char> buf(blockSize + 400);

	for(size_t off = 0; off < size;) {
		size_t tsize = Min((size_t)blockSize, size - off);
		char scratch[QLZ_SCRATCH_DECOMPRESS];
		
		int csize;
		MPIBcast() >> csize;
		if(bigEndian) ByteSwap(&csize);
		MPIBcast() >> comm::Data(&buf[0], csize);
		qlz_decompress(&buf[0], (char*)data + off, scratch);
		off += tsize;
	}
}

static void ReceiveBVH(BVH *tree) {
	int nNodes, nTris, nShTris, nMaterials;
	MPIBcast() >> nNodes >> nTris >> nShTris >> tree->depth >> nMaterials;
	if(bigEndian) {
		ByteSwap(&nNodes); ByteSwap(&nTris);
		ByteSwap(&tree->depth); ByteSwap(&nMaterials);
	}
	printf("<"); fflush(stdout);
	
	tree->nodes.resize(nNodes);
	tree->tris.resize(nTris);
	tree->shTris.resize(nShTris);

	ReceiveData(&tree->nodes[0], nNodes * sizeof(BVH::Node));
	ReceiveData(&tree->tris[0], nTris * sizeof(Triangle));
	ReceiveData(&tree->shTris[0], nShTris * sizeof(ShTriangle));
	
	if(bigEndian) {
		printf("b"); fflush(stdout);
		SwapDwords(&tree->nodes[0], nNodes * sizeof(BVH::Node) / 4);
		SwapDwords(&tree->tris[0], nTris * sizeof(Triangle) / 4);
		SwapDwords(&tree->shTris[0], nTris * sizeof(ShTriangle) / 4);
		printf("B"); fflush(stdout);
	}
	
	tree->materials.resize(nMaterials);
	for(int n = 0; n < nMaterials; n++)
		tree->materials[n].name = ReceiveString();
}

int server_main(int, char**);

#if defined(__PPC) || defined(__PPC__)
void KillSPURenderingThreads();
#endif

int node_main(int argc, char **argv) {
	int rank, numNodes, maxNodes;
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);
	MPI_Comm_size(MPI_COMM_WORLD, &numNodes);
	maxNodes = numNodes;
	const char *arch = 
#if defined(__PPC) || defined(__PPC__)
		"ppc64";
#else
	#ifdef __x86_64__
		"x86_64";
	#else
		"x86";
	#endif
#endif
	char hostName[256];
	gethostname(hostName, sizeof(hostName));
	printf("Snail v 0.20 node %d / %d arch: %s at: %s\n", rank, numNodes, arch, hostName);

	MPI_Buffer_attach(sendbuf, sizeof(sendbuf));

	vector<CompressedPart> parts(16);
	
	enum {
		blockSize = 32
	};

	while(true) {
		int resx, resy, threads = 2;
		MPIBcast() >> resx >> resy;
#ifdef __BIG_ENDIAN
		ByteSwap(&resx); ByteSwap(&resy);
#endif
		Assert(resx % blockSize == 0 && resy % blockSize == 0);

		vector<int> partCoords; {
			int nCoords;
			MPINode(0, 0) >> nCoords;
#ifdef __BIG_ENDIAN
			ByteSwap(&nCoords);
#endif
			partCoords.resize(nCoords * 4);
			if(nCoords) {
				MPINode(0, 1) >> comm::Data(&partCoords[0], nCoords * 16);
#ifdef __BIG_ENDIAN
				SwapDwords(&partCoords[0], partCoords.size());
#endif
			}
		}
		int nParts = partCoords.size() / 4;
		if(parts.size() < nParts)
			parts.resize(nParts);

		Scene<StaticTree> scene;
		vector<shading::MaterialDesc> matDescs;
		matDescs = ReceiveMatDescs();
		scene.texDict = ReceiveTexDict();
		scene.matDict = MakeMaterials(matDescs, scene.texDict);
		
		ReceiveBVH(&scene.geometry);
		scene.UpdateMaterials();
		scene.geometry.UpdateMaterialIds(scene.GetMatIdMap());

		vector<Light> lights;
		Camera cam;
		Options options;
		vector<unsigned char, AlignedAllocator<unsigned char, 16> > data(nParts * 16 + 16);
		vector<int> partOffsets(nParts); if(nParts) {
			*((int*)&data[0]) = nParts;
			memcpy(&data[4], &partCoords[0], nParts * 16);
			memset(&data[nParts * 16 + 4], 0, 12);

#ifdef __BIG_ENDIAN
			SwapDwords(&data[0], nParts * 4 + 1);
#endif

			uint zeroOffset = data.size();
			for(int n = 0; n < nParts; n++)
				partOffsets[n] = n == 0? zeroOffset : partOffsets[n - 1]
							+ partCoords[(n - 1) * 4 + 2] * partCoords[(n - 1) * 4 + 3] * 3;
			data.resize(partOffsets[nParts - 1] +
						partCoords[(nParts - 1) * 4 + 2] * partCoords[(nParts - 1) * 4 + 3] * 3);
		}
		vector<unsigned char> comprData(data.size() + 400);

		for(;;) {
			bool finish;
			MPIBcast() >> finish;
			if(finish) {
#if defined(__PPC) || defined(__PPC__)
				KillSPURenderingThreads();
#endif
				break;
			}

			int nLights, nInstances;
			float dAnimPos;

			MPIBcast() >> Pod(cam) >> nLights;
#ifdef __BIG_ENDIAN
			ByteSwap(&nLights);
			SwapDwords(&cam, sizeof(cam) / 4);
#endif

			scene.lights.resize(nLights);
			MPIBcast()	>> comm::Data(&scene.lights[0], sizeof(Light) * nLights)
						>> Pod(gVals) >> Pod(threads) >> Pod(nInstances) >> Pod(dAnimPos);
			if(nParts == 0)
				continue;

#ifdef __BIG_ENDIAN
			SwapDwords(&scene.lights[0], (sizeof(Light) * nLights) / 4);
			SwapDwords(gVals, sizeof(gVals) / 4);
			ByteSwap(&threads);
#endif
	
			Scene<DBVH> dscene;
			dscene.lights = scene.lights;
			dscene.geometry = MakeDBVH(&scene.geometry, nInstances, dAnimPos);

			TreeStats stats;
			double renderTime = GetTime();
#if defined(__PPC) || defined(__PPC__)
			stats = RenderAndSend(scene, cam, resx, resy, &data[0], &comprData[0],
						partCoords, partOffsets, options, rank, threads);
#else
			stats = nInstances > 1?
				Render(dscene, cam, resx, resy, &data[0], partCoords, partOffsets, options, rank, threads) :
				Render( scene, cam, resx, resy, &data[0], partCoords, partOffsets, options, rank, threads);
			for(int n = 0; n < sizeof(stats.timers) / sizeof(int); n++)
				stats.timers[n] = 0;

			int comprSize; {
				char scratch[QLZ_SCRATCH_COMPRESS];
				comprSize = qlz_compress((char*)&data[0], (char*)&comprData[0], data.size(), scratch);
			}
			MPINode(0, 0) << comprSize << comm::Data(&comprData[0], comprSize);
#endif
			renderTime = GetTime() - renderTime;
			int zero = 0; MPI_Bsend(&zero, sizeof(zero), MPI_CHAR, 0, 0, MPI_COMM_WORLD);

#ifdef __BIG_ENDIAN
			SwapDwords(&stats, sizeof(stats) / 4);
			ByteSwap(&renderTime);
#endif

			//TODO: jakis dziwny problem z wydajnoscia widoczny na thai.obj, jak przechodzi
			//size z klatki zajmujacej < 1024KB na taka co zajmuje wiecej (znaczny spadek wydajnosci)
			MPI_Bsend(&stats, sizeof(stats), MPI_CHAR, 0, 1, MPI_COMM_WORLD);
			MPI_Send(&renderTime, sizeof(renderTime), MPI_CHAR, 0, 2, MPI_COMM_WORLD);
		}
	}

	return 0;
}

int main(int argc,char **argv) {
	if(MPI_Init(&argc, &argv) != MPI_SUCCESS) {
		std::cout << "Error initializing mpi\n";
		return 1;
	}

	try {
		int rank;
		MPI_Comm_rank(MPI_COMM_WORLD, &rank);
		
		int ret = rank == 0?server_main(argc, argv) : node_main(argc, argv);
		MPI_Finalize();
		return ret;
	}
	catch(const Exception &ex) {
		MPI_Finalize();
		std::cout << ex.what() << '\n' << CppFilterBacktrace(ex.Backtrace()) << '\n';
		return 1;
	}
	catch(...) {
		MPI_Finalize();
		std::cout << "Unknown exception thrown.\n";
		throw;
	}
}
