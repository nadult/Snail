#include "loader.h"
#include <iostream>
#include <fstream>
#include <string.h>

using std::cout;
using std::endl;

struct Vert {
	Vert(const Vec3f &tv) :v(tv),uses(0),nrm(0,0,0) { }

	Vec3f v,nrm;
	int uses;
};
struct Tri {
	Tri(int a,int b,int c,vector<Vert> &verts) {
		i[0]=a; i[1]=b; i[2]=c;
		nrm=(verts[b].v-verts[a].v)^(verts[c].v-verts[a].v);
		nrm*=RSqrt(nrm|nrm);
		verts[a].uses++; verts[b].uses++; verts[c].uses++;
		verts[a].nrm+=nrm; verts[b].nrm+=nrm; verts[c].nrm+=nrm;
	}

	int i[3];
	Vec3f nrm;
};

void LoadWavefrontObj(const char *fileName,TriVector &out,ShadingDataVec &shadingData,float scale,uint maxTris) {

	std::filebuf fb;
	if(!fb.open (fileName,std::ios::in)) return;

	std::istream is(&fb);
	bool flipSides=0,swap=0;

	float sx=scale,sy=scale,sz=scale;

	vector<Vert> verts;
	vector<Tri> tris;

	for(;;) {
		char line[1000],type[100],a[100],b[100],c[100],d[100],e[100],f[100];
		if(!is.getline(line,1000))
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
		else if(strcmp(type,"f")==0) {
			int v[3];

			char *buf;
			buf=strchr(line,' ')+1; v[0]=atoi(buf)-1; 
			buf=strchr(buf ,' ')+1; v[1]=atoi(buf)-1;
			buf=strchr(buf ,' ')+1; v[2]=atoi(buf)-1;

			if(flipSides) tris.push_back( Tri(v[2],v[1],v[0],verts) );
			else tris.push_back( Tri(v[0],v[1],v[2],verts) );
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
	shadingData.resize(tris.size());

	for(int n=0;n<verts.size();n++) {
		verts[n].nrm*=1.0f/float(verts[n].uses);
	}

	for(int n=0;n<tris.size();n++) {
		Tri &tri=tris[n];
		out[n]=Triangle(verts[tri.i[0]].v,verts[tri.i[1]].v,verts[tri.i[2]].v);
		for(int k=0;k<3;k++) {
			if((verts[tri.i[k]].nrm|tris[n].nrm)<0.5f) Convert(tris[n].nrm,shadingData[n].nrm[k]);
			else Convert(verts[tri.i[k]].nrm,shadingData[n].nrm[k]);
		}
	}
}		


void LoadRaw(const char *filename,TriVector &out,float scale,uint maxTris) {
	FILE *f=fopen(filename,"rb");

	while(1) {
		float x[3],y[3],z[3];
		if(fscanf(f,"%f %f %f %f %f %f %f %f %f",&x[0],&y[0],&z[0],&x[1],&y[1],&z[1],&x[2],&y[2],&z[2])!=9)
			break;
		for(int n=0;n<3;n++) {
			x[n]*=scale;
			y[n]*=-scale;
			z[n]*=-scale;
		}

		out.push_back( Triangle(Vec3f(x[2],z[2],y[2]),Vec3f(x[1],z[1],y[1]),Vec3f(x[0],z[0],y[0])) );
		if(out.size()>maxTris) break;
	}

	fclose(f);
}

