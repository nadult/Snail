#include "base_scene.h"
#include <iostream>
#include <algorithm>


TriVector BaseScene::ToTriVector() const {
	TriVector out;
	
	for(int o=0;o<objects.size();o++) {
		const Object &obj=objects[o];
		
		for(int t=0;t<obj.tris.size();t++)
			out.push_back(obj.GetTriangle(t));
	}
	
	return out;
}

BBox BaseScene::GetBBox() const {
	if(objects.size()==0)
		ThrowException("Trying to compute bounding box of empty scene");
	
	BBox out=objects[0].GetBBox()*objects[0].trans;	
	for(int n=1;n<objects.size();n++)
		out+=objects[n].GetBBox()*objects[n].trans;
		
	return out;
}

void BaseScene::Transform(const Matrix<Vec4f> &mat) {
	for(int n=0;n<objects.size();n++)
		objects[n].Transform(mat);
}
void BaseScene::TransformData(const Matrix<Vec4f> &mat) {
	for(int n=0;n<objects.size();n++)
		objects[n].TransformData(mat);
}

void BaseScene::Optimize() {
	for(int n=0;n<objects.size();n++)
		objects[n].Optimize();
}

BaseScene::Object::Object(const vector<Vec3f> &tverts,const vector<Vec2f> &tuvs,const vector<Vec3f> &tnormals,
							const vector<BaseScene::IndexedTri> &ttris) {
								
	vector<int> vIMap(tverts.size()),uvIMap(tuvs.size()),nrmIMap(tnormals.size());
	
	{
		vector<char> usage(Max(Max(tverts.size(),tuvs.size()),tnormals.size()),0);
		for(int n=0;n<ttris.size();n++) {
			const BaseScene::IndexedTri &tri=ttris[n];
			for(int k=0;k<3;k++) {
				usage[tri.v[k]]|=1;
				if(tri.vt[k]>=0) usage[tri.vt[k]]|=2;
				if(tri.vn[k]>=0) usage[tri.vn[k]]|=4;
			}
		}
	
		for(int n=0;n<usage.size();n++) {
			if(usage[n]&1) {
				vIMap[n]=verts.size();
				verts.push_back(tverts[n]);
			}
			if(usage[n]&2) {
				uvIMap[n]=uvs.size();
				uvs.push_back(tuvs[n]);
			}
			if(usage[n]&4) {
				nrmIMap[n]=normals.size();
				normals.push_back(tnormals[n]);
			}
		}
	}
	
	tris.resize(ttris.size());
	for(int n=0;n<tris.size();n++) {
		const BaseScene::IndexedTri &src=ttris[n];
		BaseScene::IndexedTri &dst=tris[n];
			
		for(int k=0;k<3;k++) {
			dst.v[k]=vIMap[src.v[k]];
			dst.vt[k]=src.vt[k]>=0?uvIMap[src.vt[k]]:-1;
			dst.vn[k]=src.vn[k]>=0?nrmIMap[src.vn[k]]:-1;
		}
	}
	
	trans=Identity<>();
	bbox=BBox(&verts[0],verts.size());
	optBBox=OptBBox(bbox,Identity<>());
}

void BaseScene::Object::TransformData(const Matrix<Vec4f> &mat) {
	for(int n=0;n<verts.size();n++) verts[n]=mat*verts[n];
	for(int n=0;n<normals.size();n++) normals[n]=mat&normals[n];
	bbox=BBox(&verts[0],verts.size(),trans);
	optBBox=OptBBox(bbox,Inverse(trans));
}

void BaseScene::Object::Transform(const Matrix<Vec4f> &mat) {
	trans=mat*trans;
	bbox=BBox(&verts[0],verts.size());
	optBBox=OptBBox(bbox,Inverse(trans));
}

void BaseScene::Object::Repair() {
	for(int n=0;n<tris.size();n++) {
		IndexedTri &tri=tris[n];
		Vec3f v[3]={verts[tri.v[0]],verts[tri.v[1]],verts[tri.v[2]]};
		Vec3f nrm=(v[1]-v[0])^(v[2]-v[0]);
		if(Abs(nrm.x)<0.00000001f&&Abs(nrm.y)<0.00000001f&&Abs(nrm.z)<0.00000001f) {
			tris[n]=tris.back();
			tris.pop_back();
			n--;
		}
	}
}

namespace {
	
	struct Tri {
		Tri() { }
		Tri(int a,int b,int c,int src) :used(0),srcIdx(src) {
			if(b<a) Swap(b,a); if(c<a) Swap(c,a); if(c<b) Swap(c,b);
			v[0]=a; v[1]=b; v[2]=c;
		}
		bool operator<(const Tri &rhs) const {
			return v[0]==rhs.v[0]?v[1]==rhs.v[1]?v[2]<rhs.v[2]:v[1]<rhs.v[1]:v[0]<rhs.v[0];
		}
		
		bool used;
		int v[3],srcIdx;
	};
	
	bool Same(float a,float b) { return Abs(a-b)<Abs(a)/1000000.0f; }
	
	struct Vert {
		Vert() { }
		Vert(float xx,float yy,float zz,int idx) :srcIdx(idx),x(xx),y(yy),z(zz) { }
		bool operator<(const Vert &rhs) const {
			return Same(x,rhs.x)?Same(y,rhs.y)?Same(z,rhs.z)?srcIdx<rhs.srcIdx:z<rhs.z:y<rhs.y:x<rhs.x;
		}
		bool operator==(const Vert &rhs) const {
			return Same(x,rhs.x)&&Same(y,rhs.y)&&Same(z,rhs.z);
		}
		
		float x,y,z;
		int srcIdx,first;
	};
	
}

void BaseScene::Object::BreakToElements(vector<Object> &out) {
	vector<Vert> tVerts(verts.size());
	for(int n=0;n<verts.size();n++)
		tVerts[n]=Vert(verts[n].x,verts[n].y,verts[n].z,n);
	std::sort(tVerts.begin(),tVerts.end());
	
	tVerts[0].first=0;
	for(int n=1;n<tVerts.size();n++)
		tVerts[n].first=tVerts[n]==tVerts[n-1]?tVerts[n-1].first:n;
	
	vector<int> optVerts(verts.size());
	for(int n=0;n<tVerts.size();n++)
		optVerts[tVerts[n].srcIdx]=tVerts[n].first;
		
	vector<int> uses(tVerts.size(),0);
	int allUses=0;
	
	for(int n=0;n<tris.size();n++) {
		const IndexedTri &tri=tris[n];
		for(int v=0;v<3;v++) uses[optVerts[tri.v[v]]]++;
		allUses+=3;
	}
	
	vector<Tri> tTris(tris.size());
	for(int n=0;n<tris.size();n++) {
		const IndexedTri &tr=tris[n];
		tTris[n]=Tri(optVerts[tr.v[0]],optVerts[tr.v[1]],optVerts[tr.v[2]],n);
	}
	
	std::sort(tTris.begin(),tTris.end());
	
	{ // setting pointers for array vert->tri
		int temp=uses[0]; uses[0]=0;
		for(int n=1;n<uses.size();n++) {
			int tt=uses[n];
			uses[n]=uses[n-1]+temp;
			temp=tt;
		}
	}
	
	vector<int> vertToTri(allUses);
	for(int n=0;n<tTris.size();n++) {
		const Tri &tri=tTris[n];
		vertToTri[uses[tri.v[0]]++]=n;
		vertToTri[uses[tri.v[1]]++]=n;
		vertToTri[uses[tri.v[2]]++]=n;
	}
	
	vector<int> stack(verts.size());
	int stackPos=0;
	
	for(int t=0;t<tTris.size();t++) {
		Tri &tri=tTris[t];
		if(tri.used) continue;
		tri.used=1;
		
		vector<IndexedTri> extraction;
		for(int v=0;v<3;v++) stack[stackPos++]=tri.v[v];
		extraction.push_back(tris[tri.srcIdx]);
		
		while(stackPos) {
			int v=stack[--stackPos];
			int *triIdx=&vertToTri[v==0?0:uses[v-1]];
			int count=uses[v]-uses[v-1];
			
			for(int k=0;k<count;k++) {
				Tri &tri=tTris[triIdx[k]];
				if(tri.used) continue;
				tri.used=1;
				
				extraction.push_back(tris[tri.srcIdx]);
				for(int v=0;v<3;v++) stack[stackPos++]=tri.v[v];
			}
		}
		
//		static int c=0; c++;
		printf(","); fflush(stdout);
		out.push_back(Object(verts,uvs,normals,extraction));
		extraction.clear();
	}
}

void BaseScene::Object::Optimize() {
	FindOptimalTrans();
//	optBBox=OptBBox(&verts[0],verts.size());
	bbox=BBox(&verts[0],verts.size(),trans);
}

BaseScene::Triangle BaseScene::Object::GetTriangle(uint n) const {
	Triangle out;
	const IndexedTri &src=tris[n];

	for(int k=0;k<3;k++) out.pos[k]=verts[src.v[k]];
	out.fnrm=(out.pos[1]-out.pos[0])^(out.pos[2]-out.pos[0]);
	out.fnrm*=RSqrt(out.fnrm|out.fnrm);
	
	for(int k=0;k<3;k++) {
		out.uv[k]=src.vt[k]>=0?uvs[src.vt[k]]:Vec2f(0,0);
		out.nrm[k]=src.vn[k]>=0?normals[src.vn[k]]:out.fnrm;
	}
	
	return out;
}

TriVector BaseScene::Object::ToTriVector() const {
	TriVector out;
	
	for(int t=0;t<tris.size();t++)
		out.push_back(GetTriangle(t));
	
	return out;
}

void BaseScene::Object::FindOptimalTrans() {
	{
		Matrix<Vec4f> min=Identity<>();
		float minSum=1.0f/0.0f;
		
		enum { dx=4,dy=4,dz=4 };
		
		for(int x=0;x<dx;x++) for(int y=0;y<dy;y++) for(int z=0;z<dz;z++) {
			float ax=dx==1?0.0f:ConstPI<float>()*0.5f*float(x)/float(dx-1);
			float ay=dy==1?0.0f:ConstPI<float>()*0.5f*float(y)/float(dy-1);
			float az=dz==1?0.0f:ConstPI<float>()*0.5f*float(z)/float(dz-1);
			
			trans=Rotate(ax,ay,az);
			
			Vec3f planes[6];
			planes[0]=Vec3f(trans.x.x,trans.x.y,trans.x.z);
			planes[1]=Vec3f(trans.y.x,trans.y.y,trans.y.z);
			planes[2]=Vec3f(trans.z.x,trans.z.y,trans.z.z);
			planes[3]=-planes[0];
			planes[4]=-planes[1];
			planes[5]=-planes[2];
			
			float sum=0.0f;
			for(int n=0;n<tris.size();n++) {
				Vec3f nrm=GetTriangle(n).fnrm;
				float m=1.0f/0.0f;
				for(int k=0;k<6;k++) m=Min(m,planes[k]|nrm);
				sum+=m;
			}
			if(sum<minSum) { minSum=sum; min=trans; }
		}
		trans=min;
	}

	Matrix<Vec4f> inv=Inverse(trans);
	for(int n=0;n<verts.size();n++) verts[n]=inv*verts[n];
	for(int n=0;n<normals.size();n++) normals[n]=inv&normals[n];
}

