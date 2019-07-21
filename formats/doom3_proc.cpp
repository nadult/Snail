#include "loader.h"
#include <iostream>
#include <fstream>
#include <string.h>
#include <algorithm>
#include "base_scene.h"

namespace {

	std::map<string,string> LoadMat2TextureMap() {
		std::filebuf fb;
		if(!fb.open("scenes/doom3/materials.mtr",std::ios::in))
			FATAL("Error while opening file: scenes/doom3/materials.mtr");
		std::istream in(&fb);

		std::map<string,string> out;

		while(!in.eof()) {
			string mat,tex="",token;
			in >> mat;

			if(mat=="table") {
				in >> token;
				in >> token; //assert(token=="{");
				int pCount=1;
				while(pCount&&!in.eof()) {
					in >> token;
					if(token=="}") pCount--;
					else if(token=="{") pCount++;
				}
				continue;
			}

			in >> token; //assert(token=="{");
			int pCount=1;
			while(pCount&&!in.eof()) {
				in >> token;
				if(token=="}") pCount--;
				else if(token=="{") pCount++;
				else if(token=="diffusemap") { in >> tex; if(tex=="map") in >> tex; }
			}
					
			if(tex.substr(tex.size()>=4?tex.size()-4:0,string::npos)!=".tga"&&tex!="")
				tex=tex+".tga";
			out[mat]=tex;
		}
		return out;
	}

	BaseScene::Object ReadModel(std::istream &in,std::map<string,int> &matNames,
								const std::map<string,string> &mat2tex) {
		vector<Vec3f> verts;
		vector<Vec2f> uvs;
		vector<Vec3f> normals;
		vector<BaseScene::IndexedTri> tris;

		string name;
		int nSurfaces;

		{ string token; in >> token; assert(token=="{"); }
		in >> name >> nSurfaces;
		name=name.substr(1,name.size()-2);

		for(int n=0;n<nSurfaces;n++) {
			int lastTris=tris.size(),lastVerts=verts.size();
			int nVerts,nInds,nTris;
			string matName;

			{ string token; in >> token; assert(token=="{"); }

			in >> matName >> nVerts >> nInds;
			matName=matName.substr(1,matName.size()-2);
			nTris=nInds/3;

			if(matName.find("decals/")!=string::npos||matName.find("sfx/")!=string::npos) {
				string token;
				while(token!="}") in >> token;
				continue;
			}

			string texName="";
			if(mat2tex.find(matName)==mat2tex.end()) {
			//	std::cout << "Material not found: " << matName << '\n';
			}
			else texName=mat2tex.find(matName)->second;

			int matId;
			if(matNames.find(texName)!=matNames.end())
				matId=matNames[texName];
			else {
				matId=matNames.size();
				matNames[texName]=matId;
			}

			tris.resize(lastTris+nTris);
			verts.resize(lastVerts+nVerts);
			uvs.resize(lastVerts+nVerts);
			normals.resize(lastVerts+nVerts);

			for(int v=0;v<nVerts;v++) {
				Vec3f &pos=verts[lastVerts+v];
				Vec2f &uv=uvs[lastVerts+v];
				Vec3f &nrm=normals[lastVerts+v];
				char t1,t2;

				in >> t1 >> pos.x >> pos.z >> pos.y;
				in >> uv.x >> uv.y >> nrm.x >> nrm.z >> nrm.y >> t2;
				assert(t1=='('&&t2==')');
			}

			for(int t=0;t<nTris;t++) {
				int idx[3];
				BaseScene::IndexedTri &tri=tris[lastTris+t];
				in >> idx[0] >> idx[2] >> idx[1];
	
				tri.v[0]=tri.vt[0]=tri.vn[0]=idx[0]+lastVerts;
				tri.v[1]=tri.vt[1]=tri.vn[1]=idx[2]+lastVerts;
				tri.v[2]=tri.vt[2]=tri.vn[2]=idx[1]+lastVerts;
				tri.matId=matId;
			}

			{ string token; in >> token; assert(token=="}"); }
		}

		{ char token; in >> token; assert(token=='}'); }
		BaseScene::Object out(verts,uvs,normals,tris);
		out.SetName(name);
		return out;
	}

}

void BaseScene::LoadDoom3Proc(const string &fileName) {
	objects.clear();
	matNames.clear();
	matNames[""]=0;

	std::filebuf fb;
	if(!fb.open (fileName.c_str(),std::ios::in))
		FATAL("Error while opening file: %s",fileName.c_str());

	std::istream is(&fb);

	bool model=0,shape=0;
	int pCount=0;

	std::map<string,string> mat2tex=LoadMat2TextureMap();

	while(!is.eof()) {
		string token;
		is >> token;

		if(token=="model") 	objects.push_back(ReadModel(is,matNames,mat2tex));

		if(token=="{") pCount++;
		if(token=="}") pCount--;
	}
}

