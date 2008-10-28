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

void LoadWavefrontObj(const char *fileName,TriVector &out,ShadingDataVec &shadingData,float scale,uint maxTris) {
	std::filebuf fb;
	if(!fb.open (fileName,std::ios::in)) return;

	std::istream is(&fb);
	bool flipSides=0,swap=0;

	float sx=scale,sy=scale,sz=scale;

	vector<Vert> verts;
	vector<Vec2f> coords;
	vector<Tri> tris;

	for(;;) {
		char line[4000],type[100],a[100],b[100],c[100],d[100],e[100],f[100];
		if(!is.getline(line,4000))
			break;

		sscanf(line,"%s",type);

		if(strcmp(type,"v")==0) {
			Vec3f vert;
			sscanf(line,"%s %s %s %s",type,a,b,c);
			vert.x=atof(a)*sx;
			vert.y=-atof(b)*sy;
			vert.z=atof(c)*sz;
			if(swap) Swap(vert.y,vert.z);
			verts.push_back(Vert(vert));
		}
		else if(strcmp(type,"vt")==0) {
			Vec2f uv;
			sscanf(line,"%s %s %s",type,a,b);
			uv.x=atof(a);
			uv.y=atof(b);
			coords.push_back(uv);
		}
		else if(strcmp(type,"f")==0) {
			int v[3]={verts.size(),verts.size(),verts.size()};
			int vt[3]={0,0,0};

			char *buf;
			buf=strchr(line,' ')+1; if(buf-1) { v[0]=atoi(buf)-1;
			buf=strchr(buf ,' ')+1; if(buf-1) { v[1]=atoi(buf)-1;
			buf=strchr(buf ,' ')+1; if(buf-1) { v[2]=atoi(buf)-1;
			buf=strchr(line,'/')+1; if(buf-1) { vt[0]=atoi(buf)-1; buf=strchr(buf,' ');
			buf=strchr(buf ,'/')+1; if(buf-1) { vt[1]=atoi(buf)-1; buf=strchr(buf,' ');
			buf=strchr(buf ,'/')+1; if(buf-1) vt[2]=atoi(buf)-1; } } } } }

			for(int n=0;n<3;n++) {
				if(v[n]<0) v[n]=verts.size()+v[n];
				if(vt[n]<0) vt[n]=coords.size()+vt[n];
			}

			if(flipSides) tris.push_back( Tri(v[2],v[1],v[0],vt[2],vt[1],vt[0],verts,1) );
			else tris.push_back( Tri(v[0],v[1],v[2],vt[0],vt[1],vt[2],verts,1) );
			if(tris.size()==maxTris) break;
			
			/*char *buf=strchr(line,' ')+1;
			while(buf=strchr(buf,' ')) {
				buf++;
				v[1]=v[2];
				v[2]=atoi(buf)-1;
			}*/
		}
		else if(strcmp(type,"swap")==0) swap=1;
		else if(strcmp(type,"flip")==0) flipSides=1;
		else if(strcmp(type,"scale")==0) {
			sscanf(line,"%s %s %s %s",type,a,b,c);
			sx=sx*atof(a);
			sy=sy*atof(b);
			sz=sz*atof(c);
		}

	}
	fb.close();

	out.resize(tris.size());
	for(int n=0;n<tris.size();n++) {
		Tri &tri=tris[n];
		out[n]=Triangle(verts[tri.i[0]].v,verts[tri.i[1]].v,verts[tri.i[2]].v);
	}
	GenShadingData(verts,coords,tris,shadingData,1);

}		
