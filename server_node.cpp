#if defined(__PPC) || defined(__PPC__)
#else
	#define RANK0
#endif

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

#include <mpi.h>
#include "comm.h"

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

#ifdef __BIG_ENDIAN
	enum { bigEndian = 1 };
#else
	enum { bigEndian = 0 };
#endif

static void SendString(const string &str) {
	int size = str.size();
	MPIBcast() << size << comm::Data((void*)&str[0], size);
}

static const string ReceiveString() {
	int size;
	MPIBcast() >> size;
	if(bigEndian) ByteSwap(&size);
	string out;
	out.resize(size);
	MPIBcast() >> comm::Data(&out[0], size);
	return out;
}

static void SendTexture(const sampling::PTexture tex) {
	int width = tex->Width(), height = tex->Height();
	int size = tex->GetFormat().EvalImageSize(width, height);

	unsigned ident = tex->GetFormat().GetIdent();
	MPIBcast() << width << height << ident << size;
	MPIBcast() << comm::Data(tex->DataPointer(), size);
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
	fprintf(stderr, "finished\n");
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

static void SendBVH(BVH &tree) {
	int nNodes, nVerts, nTris, nMaterials;
	nNodes = tree.nodes.size();
	nTris = tree.elements.tris.size();
	nVerts = tree.elements.verts.size();
	nMaterials = tree.materials.size();

	MPIBcast() << nNodes << nTris << nVerts << tree.depth << nMaterials;
	MPIBcast() << comm::Data(&tree.nodes[0], nNodes * sizeof(BVH::Node));
	
	MPIBcast() << comm::Data(&tree.elements.tris[0], nTris * sizeof(CompactTris::TriIdx));
	MPIBcast() << comm::Data(&tree.elements.verts[0], nVerts * sizeof(Vec3f));
	MPIBcast() << comm::Data(&tree.elements.shData[0], nVerts * sizeof(CompactTris::ShData));

	for(int n = 0; n < nMaterials; n++)
		SendString(tree.materials[n].name);
}

static void ReceiveBVH(BVH *tree) {
	int nNodes, nTris, nVerts, nMaterials;
	MPIBcast() >> nNodes >> nTris >> nVerts >> tree->depth >> nMaterials;
	if(bigEndian) {
		ByteSwap(&nNodes); ByteSwap(&nTris);
		ByteSwap(&nVerts); ByteSwap(&tree->depth);
		ByteSwap(&nMaterials);
	}
	
	tree->nodes.resize(nNodes);
	MPIBcast() >> comm::Data(&tree->nodes[0], nNodes * sizeof(BVH::Node));
	if(bigEndian) SwapDwords(&tree->nodes[0], nNodes * sizeof(BVH::Node) / 4);
	
	tree->elements.tris.resize(nTris);
	tree->elements.verts.resize(nVerts);
	tree->elements.shData.resize(nVerts);

	MPIBcast() >> comm::Data(&tree->elements.tris[0], nTris * sizeof(CompactTris::TriIdx));
	if(bigEndian) SwapDwords(&tree->elements.tris[0], nTris * sizeof(CompactTris::TriIdx) / 4);

	MPIBcast() >> comm::Data(&tree->elements.verts[0], nVerts * sizeof(Vec3f));
	if(bigEndian) SwapDwords(&tree->elements.verts[0], nVerts * sizeof(Vec3f) / 4);

	MPIBcast() >> comm::Data(&tree->elements.shData[0], nVerts * sizeof(CompactTris::ShData));
	if(bigEndian) SwapDwords(&tree->elements.shData[0], nVerts * sizeof(CompactTris::ShData) / 4);
	tree->UpdateCache();
	
	tree->materials.resize(nMaterials);
	for(int n = 0; n < nMaterials; n++)
		tree->materials[n].name = ReceiveString();
}

static int server_main(int argc, char **argv) {
	int rank, numNodes, maxNodes;
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);
	MPI_Comm_size(MPI_COMM_WORLD, &numNodes);
	maxNodes = numNodes;
	const char *arch = 
#if defined(__PPC) || defined(__PPC__)
		"ppc64";
#else
		"x86_64";
#endif
	printf("Snail v 0.20 node %d / %d arch: %s\n", rank, numNodes, arch);

	MPI_Buffer_attach(sendbuf, sizeof(sendbuf));

	enum { lineHeight = 16 };

	vector<CompressedPart> parts(16);

	while(true) {
		if(rank == 0) {
#ifdef RANK0
			Socket socket;
			socket.Accept(20002);

			int resx, resy, threads = 2;
			char tsceneName[256];
			socket >> Pod(resx) >> Pod(resy) >> Pod(tsceneName);
			string sceneName(tsceneName);
			string texPath = sceneName; {
				int pos = texPath.rfind('/');
				if(pos == string::npos) texPath = "";
				else texPath.resize(pos + 1);
			}
			
			for(int r = 1; r < numNodes; r++)
				MPINode(r, 0) << resx << resy;

			Scene<StaticTree> scene;
			//TODO: try catch
			Loader(string("dump/") + sceneName) & scene.geometry;
			scene.geometry.UpdateCache();
			
			vector<shading::MaterialDesc> matDescs;
			if(sceneName.substr(sceneName.size() - 4) == ".obj") {
				string mtlFile = "scenes/" + sceneName.substr(0, sceneName.size() - 3) + "mtl";
				matDescs = shading::LoadMaterialDescs(mtlFile);
			}
			SendMatDescs(matDescs);
			scene.texDict = LoadTextures(matDescs, "scenes/" + texPath);
			SendTexDict(scene.texDict);
			scene.matDict = shading::MakeMaterials(matDescs, scene.texDict);

			scene.UpdateMaterials();
			scene.geometry.UpdateMaterialIds(scene.GetMatIdMap());

			SendBVH(scene.geometry);

			Vec3f sceneCenter = scene.geometry.GetBBox().Center();
			Vec3f sceneSize = scene.geometry.GetBBox().Size();
			socket << Pod(sceneCenter) << Pod(sceneSize);

			gfxlib::Texture image(resx, resy, gfxlib::TI_R8G8B8);
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
			//	dscene.geometry = MakeDBVH(&scene.geometry);
				dscene.lights = scene.lights;
				buildTime = GetTime() - buildTime;

				int nPixels = w * h, received = 0;
				double renderTime = GetTime();
				TreeStats stats = 
					Render(scene, cam, image, rank, numNodes, lineHeight, options, threads);
				renderTime = GetTime() - renderTime;
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

				double renderTimes[32];
				renderTimes[0] = renderTime;

				for(int r = 1; r < numNodes; r++) {
					TreeStats nodeStats;
					MPIAnyNode(0, 1) >> Pod(nodeStats);
					double renderTime;
					MPINode(r, 2) >> Pod(renderTime);
					if(r < 32) renderTimes[r] = renderTime;
					stats += nodeStats;
				}
				socket << Pod(stats) << Pod(buildTime) << Pod(numNodes) << Pod(renderTimes);
				time = GetTime() - time;

				printf("%s at %dx%d on %d nodes, each with %d threads: %.2f ms\n",
						sceneName.c_str(), resx, resy, numNodes, threads, time * 1000.0);
			}
#else
			ThrowException("Node with rank 0 should work on x86\n");
#endif
		}
		else {
			int resx, resy, threads = 2;
			MPINode(0, 0) >> resx >> resy;
#ifdef __BIG_ENDIAN
			ByteSwap(&resx); ByteSwap(&resy);
#endif

			Scene<StaticTree> scene;
			vector<shading::MaterialDesc> matDescs;
			matDescs = ReceiveMatDescs();
			scene.texDict = ReceiveTexDict();
			scene.matDict = MakeMaterials(matDescs, scene.texDict);
			
			ReceiveBVH(&scene.geometry);
			scene.UpdateMaterials();
			scene.geometry.UpdateMaterialIds(scene.GetMatIdMap());
		//	scene.geometry.PrintInfo();

			gfxlib::Texture image(resx, resy, gfxlib::TI_R8G8B8);
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
#ifdef __BIG_ENDIAN
				ByteSwap(&nLights);
				ByteSwap(&cam.plane_dist);
				ByteSwap(&cam.ang);
				ByteSwap(&cam.pitch);
				ByteSwap(&cam.pos);
#endif

				scene.lights.resize(nLights);
				MPINode(0, 3) >> comm::Data(&scene.lights[0], sizeof(Light) * nLights)
				   	>> Pod(gVals) >> Pod(threads);
#ifdef __BIG_ENDIAN
				for(int n = 0; n < scene.lights.size(); n++) {
					ByteSwap(&scene.lights[n].pos);
					ByteSwap(&scene.lights[n].color);
					ByteSwap(&scene.lights[n].radius);
					ByteSwap(&scene.lights[n].radSq);
					ByteSwap(&scene.lights[n].iRadius);
				}
				for(int n = 0; n < sizeof(gVals) / 4; n++)
					ByteSwap(&gVals[n]);
				ByteSwap(&threads);
#endif
		
				Scene<DBVH> dscene;
				dscene.lights = scene.lights;
			//	dscene.geometry = MakeDBVH(&scene.geometry);

				//TODO: rendering i kompresja razem: watek zaraz po zrenderowaniu
				//dodaje do kolejki zadan nowe zadanie kompresji?
				double renderTime = GetTime();
				TreeStats stats = 
					Render(scene, cam, image, rank, numNodes, lineHeight, options, threads);
				renderTime = GetTime() - renderTime;
				int nParts = CompressParts(image, rank, numNodes, lineHeight, parts, threads);

				for(int p = 0; p < nParts; p++) {
					CompressedPart::Info tinfo = parts[p].info;
#ifdef __BIG_ENDIAN
					ByteSwap(&tinfo.x);
					ByteSwap(&tinfo.y);
					ByteSwap(&tinfo.w);
					ByteSwap(&tinfo.h);
					ByteSwap(&tinfo.size);
#endif
					MPI_Bsend(&tinfo, sizeof(CompressedPart::Info), MPI_CHAR, 0, 0, MPI_COMM_WORLD);
					MPI_Bsend(&parts[p].data[0], parts[p].info.size, MPI_CHAR, 0, 0, MPI_COMM_WORLD);
				}
#ifdef __BIG_ENDIAN
				for(int n = 0; n < sizeof(stats) / 4; n++)
					ByteSwap(((int*)&stats) + n);
				ByteSwap(&renderTime);
#endif

				MPI_Bsend(&stats, sizeof(stats), MPI_CHAR, 0, 1, MPI_COMM_WORLD);
				MPI_Send(&renderTime, sizeof(renderTime), MPI_CHAR, 0, 2, MPI_COMM_WORLD);
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
