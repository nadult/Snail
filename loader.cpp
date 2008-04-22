#include "loader.h"
#include <iostream>
#include <fstream>
#include <string.h>

using std::cout;
using std::endl;


void LoadWavefrontObj(const char *fileName,vector<Triangle> &out,float scale) {
	std::filebuf fb;
	if(!fb.open (fileName,std::ios::in))
		return;

	std::istream is(&fb);
	vector<Vec3f> verts,normals,tex;
	bool flipSides=0;

	float sx=scale,sy=scale,sz=scale;

	int count=0;
	for(;;) {
		char line[1000],type[100],a[100],b[100],c[100],d[100];
		if(!is.getline(line,1000))
			break;

		sscanf(line,"%s",type);

		if(strcmp(type,"v")==0) {
			Vec3f vert;
			sscanf(line,"%s %s %s %s",type,a,b,c);
			vert.x=atof(a)*sx;
			vert.y=-atof(b)*sy;
			vert.z=atof(c)*sz;
			verts.push_back(vert);
		}
		else if(strcmp(type,"f")==0) {
			if(count++>750000) break;
			int v[3];

			char *buf;
			buf=strchr(line,' ')+1; v[0]=atoi(buf)-1; 
			buf=strchr(buf ,' ')+1; v[1]=atoi(buf)-1;
			buf=strchr(buf ,' ')+1; v[2]=atoi(buf)-1;
			if(flipSides) out.push_back(Triangle(verts[v[2]],verts[v[1]],verts[v[0]]));
			else out.push_back(Triangle(verts[v[0]],verts[v[1]],verts[v[2]]));
			
			/*char *buf=strchr(line,' ')+1;
			while(buf=strchr(buf,' ')) {
				buf++;
				v[1]=v[2];
				v[2]=atoi(buf)-1;
			}*/
		}
		else if(strcmp(type,"flip")==0)
			flipSides=1;
		else if(strcmp(type,"scale")==0) {
			sscanf(line,"%s %s %s %s",type,a,b,c);
			sx=sx*atof(a);
			sy=sy*atof(b);
			sz=sz*atof(c);
		}

	}
	fb.close();
	printf("Done loading %s\n",fileName);
}		

void LoadRaw(const char *filename,vector<Triangle> &out,float scale) {
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

		out.push_back(Triangle(Vec3f(x[2],z[2],y[2]),Vec3f(x[1],z[1],y[1]),Vec3f(x[0],z[0],y[0])));
	}

	fclose(f);
}

