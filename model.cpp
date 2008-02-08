#include "rtracer.h"
#include <iostream>
#include <fstream>

using std::cout;
using std::endl;


void LoadModel(const char *fileName,vector<Object> &out,float scale) {
	std::filebuf fb;
	fb.open (fileName,std::ios::in);
	std::istream is(&fb);
	vector<Vec3f> verts,normals,tex;

	int count=0;
	for(;;) {
		char line[1000],type[100],a[100],b[100],c[100],d[100];
		is.getline(line,1000);

		int num=sscanf(line,"%s %s %s %s %s",type,a,b,c,d);

		if(strcmp(type,"v")==0) {
			Vec3f vert;
			vert.X()=atof(a)*scale;
			vert.Y()=atof(b)*scale;
			vert.Z()=atof(c)*scale;
			verts.push_back(vert);
		}
		else if(strcmp(type,"f")==0) {
			if(count++>10000) break;
			int v[4];
			v[0]=atoi(a)-1;
			v[1]=atoi(b)-1;
			v[2]=atoi(c)-1;
			if(num==5) v[3]=atoi(d)-1;
			
			out.push_back(Triangle(verts[v[0]],verts[v[1]],verts[v[2]]));
			if(num==5) out.push_back(Triangle(verts[v[0]],verts[v[2]],verts[v[3]]));
		}
		else if(strcmp(type,"end")==0)
			break;
	}
	fb.close();
	printf("Done loading %s\n",fileName);
}		

