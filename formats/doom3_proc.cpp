#include "loader.h"
#include <iostream>
#include <fstream>
#include <string.h>

using std::cout;
using std::endl;

namespace {

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
	}

	int atoi(const string &str) { return ::atoi(str.c_str()); }
	float atof(const string &str) { return ::atof(str.c_str()); }

}

struct TVert { Vec3f pos,nrm; Vec2f uv; };

void LoadProc(const string &fileName,TriVector &out,ShadingDataVec &shData,float scale,uint maxTris) {
	FILE *f=fopen(fileName.c_str(),"rb");

	vector<TVert> verts;
	bool model=0,shape=0;

	while(!feof(f)) {
		char token[128];
		fscanf(f,"%s ",token);

		if(strcmp(token,"{")==0&&!model) {
			int nVerts,nInds;
			char tex[256];
			fscanf(f,"%s %d %d ",token,&nVerts,&nInds);
			strcpy(tex,token+1); tex[strlen(tex)-1]=0;

			if(verts.size()<nVerts) verts.resize(nVerts);		
			for(int v=0;v<nVerts;v++) {
				TVert vert; char t1,t2;
				fscanf(f,"%c %f %f %f %f %f %f %f %f %c ",&t1,
						&vert.pos.x,&vert.pos.z,&vert.pos.y,&vert.uv.x,&vert.uv.y,
						&vert.nrm.x,&vert.nrm.z,&vert.nrm.y,&t2);
				verts[v]=vert;
			}

			int nTris=nInds/3;
			for(int t=0;t<nTris;t++) {
				int idx[3];
				fscanf(f,"%d %d %d ",&idx[0],&idx[2],&idx[1]);

				out.push_back(Triangle(verts[idx[0]].pos,verts[idx[1]].pos,verts[idx[2]].pos));
				shData.push_back(ShadingData());
				shData.back().nrm[0]=-verts[idx[0]].nrm;
				shData.back().nrm[1]=-verts[idx[1]].nrm;
				shData.back().nrm[2]=-verts[idx[2]].nrm;
				shData.back().uv[0]=verts[idx[0]].uv;
				shData.back().uv[1]=verts[idx[1]].uv;
				shData.back().uv[2]=verts[idx[2]].uv;
			}
		}
		if(strcmp(token,"model")==0) model=1; else model=0;
		if(strcmp(token,"}")==0) shape=0;

		if(strcmp(token,"shadowModel")==0) {
			while(strcmp(token,"}")!=0) fscanf(f,"%s ",token);
		}
	}

	fclose(f);
}

