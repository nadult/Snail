#include "base_scene.h"
#include <iostream>


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
	FindOptimalTrans();
	
	UpdateBox();
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

void BaseScene::Object::UpdateBox() {
	if(verts.size()==0) {
		bbox=BBox(Vec3f(0,0,0),Vec3f(0,0,0));
		return;
	}
	
	bbox.min=bbox.max=verts[0];
	for(int n=1;n<verts.size();n++) {
		Vec3f vert=verts[n];
		bbox.min=VMin(bbox.min,vert);
		bbox.max=VMax(bbox.max,vert);
	}
}

void BaseScene::Object::FindOptimalTrans() {
	{
		Matrix<Vec4f> min;
		float minSum=1.0f/0.0f;
		
		for(int n=0;n<32;n++) {
			float ang=ConstPI<float>()*0.5f*float(n)/31.0f;
			trans=RotateY(ang);
			
			Vec3f planes[6];
			for(int k=0;k<4;k++) planes[k]=RotateY(ang+k*ConstPI<float>()*0.5f)*Vec3f(1,0,0);
			planes[4]=Vec3f(0,1,0); planes[5]=Vec3f(0,-1,0);
			
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
	for(int n=0;n<verts.size();n++) {
		verts[n]=inv*verts[n];
		//TODO przeksztalcic normalne
	}
}
