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
		Tri(int a,int b,int c,vector<Vert> &verts,bool generate,bool neg=0) {
			i[0]=a; i[1]=b; i[2]=c;
			nrm=(verts[b].v-verts[a].v)^(verts[c].v-verts[a].v);
			nrm*=RSqrt(nrm|nrm);
			if(neg) nrm=-nrm;

			if(generate) {
				verts[a].nrm+=nrm;
				verts[b].nrm+=nrm;
				verts[c].nrm+=nrm;
			}
		}

		int i[3];
		Vec3f nrm;
	};

	struct Mat {
		Mat(const string &n,const string &t,bool ts) :name(n),texture(t),twoSided(ts) { }

		bool twoSided;
		string name,texture;
	};

	void GenShadingData(vector<Vert> &verts,vector<Tri> &tris,ShadingDataVec &shadingData,bool generate) {
		shadingData.resize(tris.size());

		for(int n=0;n<verts.size();n++)
			verts[n].nrm*=RSqrt(verts[n].nrm|verts[n].nrm);

		for(int n=0;n<tris.size();n++) {
			Tri &tri=tris[n];
			for(int k=0;k<3;k++) {
				if((verts[tri.i[k]].nrm|tris[n].nrm)<0.5f||!generate) Convert(tris[n].nrm,shadingData[n].nrm[k]);
				else Convert(verts[tri.i[k]].nrm,shadingData[n].nrm[k]);
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

			if(flipSides) tris.push_back( Tri(v[2],v[1],v[0],verts,1) );
			else tris.push_back( Tri(v[0],v[1],v[2],verts,1) );
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
	GenShadingData(verts,tris,shadingData,1);

}		

void LoadV3O(string fileName,TriVector &out,ShadingDataVec &shadingData,float scale,uint maxTris) {
	std::filebuf fb;
	if(!fb.open (fileName.c_str(),std::ios::in)) throw Exception(string("Cannot open file ")+fileName);

	std::istream is(&fb);
	vector<string> tokens;
	tokens.reserve(128);

	vector<Vert> verts;
	vector<Tri> tris;
	vector<Mat> mats;
	mats.push_back(Mat("default","",0));
	scale *= 0.001f;

	int idxAdd=0;

	for(;;) {
		char line[16*1024];
		if(!is.getline(line,16*1024)) break;

		if(line[0]=='/'&&line[1]=='/') continue;

		tokens.clear();
		for(int last=0,n=0,end=strlen(line);n<=end;n++) if(line[n]==','||line[n]==' '||n==end) {
			line[n]=0;
			if(n-last>0) tokens.push_back(string(line+last));
			last=n+1;
		}
		if(tokens.size()<1) continue;

		if(tokens[0]=="SRF") {
			if(tokens.size()<12) continue;
			mats.push_back(Mat(tokens[1],tokens[5],atoi(tokens[11])));
		}
		if(tokens[0]=="D") {
			enum { A=0, B=2, C=1 };
			if(tokens.size()<13) continue;
			verts.push_back(Vec3f(atof(tokens[1+A]),-atof(tokens[1+B]),atof(tokens[1+C])));
			verts.back().v *= scale;
		}
		if(tokens[0]=="P") {
			if(tokens.size()<5) continue;
			if(atoi(tokens[1])!=3) continue;
			int v[3]={ atoi(tokens[2])-1+idxAdd, atoi(tokens[3])-1+idxAdd, atoi(tokens[4])-1+idxAdd };
			int nMat=atoi(tokens[9]);
			const Mat &mat=mats[nMat<1||nMat>=mats.size()?0:nMat];

			tris.push_back(Tri(v[0],v[1],v[2],verts,0,1));
			if(mat.twoSided) tris.push_back(Tri(v[1],v[0],v[2],verts,0,1));
		}
		if(tokens[0]=="TLS") {
			int count=atoi(tokens[1])/3;

			for(int n=0;n<count;n++) {
				int v[3]={ atoi(tokens[2+n*3])-1, atoi(tokens[3+n*3])-1, atoi(tokens[4+n*3])-1 };
				tris.push_back(Tri(v[0]+idxAdd,v[1]+idxAdd,v[2]+idxAdd,verts,0,1));
			}
		}
		if(tokens[0]=="HMAP") {
			continue;

			string hmapFile=string("scenes/desperados/")+tokens[1];
			for(int n=0;n<hmapFile.length();n++)
				if(hmapFile[n]=='\\') hmapFile[n]='/';

			try {
				Loader ldr(hmapFile);
				u16 width,height;
				ldr &width & height;
				ldr.Skip(15);

				idxAdd=verts.size();

				vector<i16> hmap2(width*height);
				vector<i16> hmap(width*height/4);
				ldr.Data(&hmap2[0],width*height*2);
				width/=2; height/=2;

				for(int y=0;y<height;y++) for(int x=0;x<width;x++) hmap[x+y*width]=hmap2[x*2+y*width*4];

				Vec3f hscale=Vec3f(1.0f/float(width),0.25f/65535.0f,1.0f/float(height))*1000000.0f*scale;
				for(int y=0;y<height;y++) for(int x=0;x<width;x++)
					verts.push_back(hscale*Vec3f(x-width/2,-hmap[x+y*width],y-height/2));

				for(int y=0;y<height-1;y++) for(int x=0;x<width-1;x++) {
					tris.push_back(Tri(x+y*width+idxAdd,x+1+y*width+idxAdd,x+(y+1)*width+idxAdd,verts,0,1));
					tris.push_back(Tri(x+(y+1)*width+idxAdd,x+1+y*width+idxAdd,x+1+(y+1)*width+idxAdd,verts,0,1));
				}
				idxAdd=verts.size();

			} catch(const Exception &ex) {
				cout << ex.what() << '\n';
				continue;
			}
		}
	}

	out.resize(tris.size());
	for(int n=0;n<tris.size();n++) {
		Tri &tri=tris[n];
		out[n]=Triangle(verts[tri.i[1]].v,verts[tri.i[0]].v,verts[tri.i[2]].v);
	}
	GenShadingData(verts,tris,shadingData,0);
}

void LoadModel(const string &fileName,TriVector &out,ShadingDataVec &shadingData,float scale,uint maxTris) {
	string ext; {
		ext=fileName.substr(Max(0,int(fileName.length())-4));
		for(int n=0,end=ext.length();n<end;n++) ext[n]=tolower(ext[n]);
	}

	if(ext==".obj") LoadWavefrontObj(fileName.c_str(),out,shadingData,scale,maxTris);
	else if(ext==".v3o"||ext==".v3d") LoadV3O(fileName.c_str(),out,shadingData,scale,maxTris);
	else throw Exception(string("Format ")+ext+" not supported");
}

