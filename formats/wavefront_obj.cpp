#include "base_scene.h"
#include <iostream>
#include <fstream>
#include <string.h>

using std::cout;
using std::endl;

namespace {

	/*
	struct Vert {
		Vert(const Vec3f &tv) :v(tv),nrm(0,0,0) { }

		Vec3f v,nrm;
	};
	struct Tri {
		Tri(int a,int b,int c,int ta,int tb,int tc,vector<Vert> &verts,bool generate,bool neg=0) {
			i[0]=a; i[1]=b; i[2]=c;
			t[0]=ta; t[1]=tb; t[2]=tc;
			nrm=(verts[b].v-verts[a].v)^(verts[c].v-verts[a].v);
			nrm*=RSqrt(nrm|nrm);
			if(neg) nrm=-nrm;

			if(generate) {
				verts[a].nrm+=nrm;
				verts[b].nrm+=nrm;
				verts[c].nrm+=nrm;
			}
		}

		int i[3],t[3];
		Vec3f nrm;
	};

	struct Mat {
		Mat(const string &n,const string &t,bool ts) :name(n),texture(t),twoSided(ts) { }

		bool twoSided;
		string name,texture;
	};


	void GenShadingData(vector<Vert> &verts,vector<Vec2f> &coords,vector<Tri> &tris,
						ShadingDataVec &shadingData,bool generate) {
		shadingData.resize(tris.size());

		for(int n=0;n<verts.size();n++)
			verts[n].nrm*=RSqrt(verts[n].nrm|verts[n].nrm);

		for(int n=0;n<tris.size();n++) {
			Tri &tri=tris[n];
			for(int k=0;k<3;k++) {
				if((verts[tri.i[k]].nrm|tris[n].nrm)<0.5f||!generate) Convert(tris[n].nrm,shadingData[n].nrm[k]);
				else Convert(verts[tri.i[k]].nrm,shadingData[n].nrm[k]);
				shadingData[n].uv[k]=coords.size()>tri.t[k]?coords[tri.t[k]]:Vec2f(0.0f,0.0f);
			}
		}
	}*/

	int atoi(const string &str) { return ::atoi(str.c_str()); }
	float atof(const string &str) { return ::atof(str.c_str()); }

}

void BaseScene::LoadWavefrontObj(const string &fileName) {
	objects.clear();
	
	std::filebuf fb;
	if(!fb.open (fileName.c_str(),std::ios::in))
		ThrowException("Error while opening: ", fileName);

	std::istream is(&fb);
	
	vector<Vec3f> verts;
	vector<Vec2f> uvs;
	vector<Vec3f> normals;
	vector<BaseScene::IndexedTri> tris;

	matNames.clear();
	matNames[""]=0;
	int lastMatId=0;

	for(;;) {
		char line[2000], type[2000], a[2000], b[2000], c[2000], d[2000], e[2000], f[2000];
		if(!is.getline(line,2000))
			break;

		sscanf(line, "%s", type);

		if(strcmp(type,"o")==0) {
		//	if(tris.size()>0) {
		//		objects.push_back(BaseScene::Object(verts,uvs,normals,tris));
		//		objects.back().name = strpbrk(line," \t") + 1;
		//		tris.clear();
		//	}
		}
		else if(strcmp(type,"v")==0) {
			Vec3f vert;
			sscanf(line,"%s %f %f %f", type, &vert.x, &vert.y, &vert.z);
			verts.push_back(vert);
		}
		else if(strcmp(type,"vt")==0) {
			Vec2f uv; float w;
			sscanf(line,"%s %f %f %f", type, &uv.x, &uv.y, &w);
			uvs.push_back(uv);
		}
		else if(strcmp(type,"vn")==0) {
			Vec3f nrm;
			sscanf(line,"%s %f %f %f", type, &nrm.x, &nrm.y, &nrm.z);
			normals.push_back(nrm);
		}
		else if(strcmp(type,"f")==0) {
			char *p[4];
			p[0] = strpbrk(line, " \t") + 1; while(p[0][0]==' ') p[0]++;
			p[1] = strpbrk(p[0], " \t") + 1; p[1][-1] = 0;
			p[2] = strpbrk(p[1], " \t") + 1; p[2][-1] = 0;
			p[3] = strpbrk(p[2], " \t");

			BaseScene::IndexedTri tri;
			tri.matId = lastMatId;
	REPEAT_TRI:

			for(int k = 0; k < 3; k++) {
				tri.v[k] = atoi(p[k]);

				if(tri.v[k] < 0) tri.v[k] = verts.size() + tri.v[k] + 1;
				tri.vt[k] = tri.vn[k] = 0;
				
				char *puv = strchr(p[k], '/');
				
				if(puv) {
					char *pnrm = strchr(puv + 1, '/');
					if(pnrm != puv + 1) {
						tri.vt[k] = atoi(puv + 1);
						if(tri.vt[k] < 0) tri.vt[k] = uvs.size() + tri.vt[k];
					}
					if(pnrm && pnrm[1]) {
						tri.vn[k] = atoi(pnrm + 1);
						if(tri.vn[k] < 0) tri.vn[k] = normals.size() + tri.vn[k];
					}
				}
				
				tri.v[k]--;
				tri.vt[k]--;
				tri.vn[k]--;

				if(tri.v[k] >= int(verts.size()) || tri.v[k] < 0) {
					std::cout << line << '\n';
					ThrowException("Wrong vertex index: ",tri.v[k],"/",int(verts.size()));
				}
				if(tri.vt[k] >= int(uvs.size()))
					ThrowException("Wrong tex-coord index: ",tri.vt[k],"/",int(uvs.size()));
				if(tri.vn[k] >= int(normals.size()))
					ThrowException("Wrong normal index",tri.vn[k],"/",int(normals.size()));
			}
			
			tris.push_back(tri);
			if(p[3] && strpbrk(p[3], "0123456789")) {
				p[1] = p[3];
				Swap(p[0], p[2]);
				p[3] = 0;
				goto REPEAT_TRI;
			}
		}
		else if(strcmp(type,"usemtl")==0) {
			sscanf(line,"%s %s",type,a);
			string matName=a;
			std::map<string,int>::iterator it=matNames.find(matName);
			if(it == matNames.end()) {
				lastMatId=matNames.size();
				matNames[matName]=lastMatId;
			}
			else lastMatId=it->second;
		}
	}


	fb.close();
	if(tris.size())
		objects.push_back(BaseScene::Object(verts,uvs,normals,tris));
	for(int n=0;n<objects.size();n++)
		objects[n].Repair();

	int nTris=0,nVerts=0;
	for(int n=0;n<objects.size();n++) {
		nTris+=objects[n].tris.size();
		nVerts+=objects[n].verts.size();
	}
	std::cout << "Done loading. Verts:" << nVerts << " Tris:" << nTris << '\n';

/*	out.resize(tris.size());
	for(int n=0;n<tris.size();n++) {
		Tri &tri=tris[n];
		out[n]=Triangle(verts[tri.i[0]].v,verts[tri.i[1]].v,verts[tri.i[2]].v);
	}
	GenShadingData(verts,coords,tris,shadingData,1); */
}

void BaseScene::SaveWavefrontObj(const string &fileName) const {
	std::filebuf fb;
	if(!fb.open (fileName.c_str(),std::ios::out)) return;

	std::ostream os(&fb);
	
	for(int n=0;n<objects.size();n++) {
		const Object &obj=objects[n];
		os << "o " << (obj.name==""?Stringize("object_",n):obj.name) << '\n';
		
		for(int f=0;f<obj.tris.size();f++) {
			const IndexedTri &tri=obj.tris[f];
			Vec3f vert=obj.trans * obj.verts[tri.v[0]];
			os << "v " << vert.x << ' ' << -vert.y << ' ' << vert.z << '\n';
			vert=obj.trans * obj.verts[tri.v[1]];
			os << "v " << vert.x << ' ' << -vert.y << ' ' << vert.z << '\n';
			vert=obj.trans * obj.verts[tri.v[2]];
			os << "v " << vert.x << ' ' << -vert.y << ' ' << vert.z << '\n';
			os << "f -3 -2 -1\n";
		}
	}
	
	fb.close();
}
