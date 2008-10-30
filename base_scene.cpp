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

void BaseScene::Transform(const Matrix<Vec4f> &mat) {
	for(int n=0;n<objects.size();n++)
		objects[n].Transform(mat);
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

void BaseScene::Object::Optimize() {
	FindOptimalTrans();
	optBBox=OptBBox(&verts[0],verts.size());
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
		Matrix<Vec4f> min;
		float minSum=1.0f/0.0f;
		
		enum { dx=8,dy=8,dz=8 };
		
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

