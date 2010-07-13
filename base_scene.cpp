#include "base_scene.h"
#include <iostream>
#include <algorithm>


CompactTris BaseScene::ToCompactTris() const {
	CompactTris out;

	for(size_t o = 0; o < objects.size(); o++) {
		const Matrix<Vec4f> &trans = objects[o].trans;
		size_t verts = out.verts.size();
		size_t inds = out.tris.size();
		
		CompactTris obj = objects[o].ToCompactTris();
		int newVerts = verts + obj.verts.size();
		out.verts.resize(newVerts);
		out.normals.resize(newVerts);

		for(size_t n = 0; n < obj.verts.size(); n++) {
			out.verts[verts + n] = trans * obj.verts[n];
			out.normals[verts + n] = trans & obj.normals[n];
		}

		out.tris.resize(out.tris.size() + obj.tris.size());
		for(size_t n = 0; n < obj.tris.size(); n++) {
			CompactTris::TriIdx &idx = out.tris[n + inds];
			idx = obj.tris[n];

			idx.v1 += verts;
			idx.v2 += verts;
			idx.v3 += verts;
		}

	}

	return out;
}

TriangleVector BaseScene::ToTriangleVector() const {
	TriangleVector out;

	for(int o=0;o<objects.size();o++) {
		const Matrix<Vec4f> &trans=objects[o].trans;
		int verts=out.verts.size();
		int inds=out.tris.size();
		
		TriangleVector obj=objects[o].ToTriangleVector();
		int newVerts=verts+obj.verts.size();

		out.verts.resize(newVerts);

		for(int n=0;n<obj.verts.size();n++) {
			TriangleVector::Vert &vert=out.verts[n+verts];;
			vert.pos=trans*obj.verts[n].pos;
			vert.uv=obj.verts[n].uv;
			vert.nrm=trans&obj.verts[n].nrm;
		}

		out.tris.resize(out.tris.size()+obj.tris.size());
		for(int n=0;n<obj.tris.size();n++) {
			TriangleVector::TriIdx &idx=out.tris[n+inds];
			idx=obj.tris[n];

			idx.v1+=verts;
			idx.v2+=verts;
			idx.v3+=verts;
		}
	}

	out.triAccels.resize(out.tris.size());

	for(int n=0;n<out.tris.size();n++) {
		const TriangleVector::TriIdx &idx=out.tris[n];
		out.triAccels[n]=TriAccel(out.verts[idx.v1].pos,out.verts[idx.v2].pos,out.verts[idx.v3].pos);
	}
	return out;
}

TriVector BaseScene::ToTriVector() const {
	TriVector out;
	
	for(int o=0;o<objects.size();o++) {
		const Object &obj=objects[o];
		
		for(int t=0;t<obj.tris.size();t++)
			out.push_back(obj.GetTriangle(t));
	}
	
	return out;
}

ShTriVector BaseScene::ToShTriVector() const {
	ShTriVector out;

	for(int o=0;o<objects.size();o++) {
		ShTriVector t=objects[o].ToShTriVector();
		for(int n=0;n<t.size();n++) out.push_back(t[n]);
	}

	return out;
}

void BaseScene::FlipNormals() {
	for(int o=0;o<objects.size();o++)
		objects[o].FlipNormals();
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
			if(usage[n] & 1) {
				vIMap[n]=verts.size();
				verts.push_back(tverts[n]);
			}
			if(usage[n] & 2) {
				uvIMap[n]=uvs.size();
				uvs.push_back(tuvs[n]);
			}
			if(usage[n] & 4) {
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
		dst.matId=src.matId;
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

void BaseScene::Object::BreakToElements(vector<Object> &out) const {
	if(!verts.size()||!tris.size()) return;

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
	
	vector<int> stack(verts.size()+100000);
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
	out.matId=src.matId;
	
	return out;
}


void BaseScene::Object::FlipNormals() {
	for(int t=0;t<tris.size();t++) {
		IndexedTri &tri=tris[t];
		Swap(tri.v[0],tri.v[1]);
		Swap(tri.vt[0],tri.vt[1]);
		Swap(tri.vn[0],tri.vn[1]);
	}
	for(int n=0;n<normals.size();n++)
		normals[n]=-normals[n];
}

TriVector BaseScene::Object::ToTriVector() const {
	TriVector out;
	for(int t=0;t<tris.size();t++)
		out.push_back(GetTriangle(t));
	return out;
}

namespace {

	struct Idx {
		int v,u,n,dstIdx;
		Idx() { }
		Idx(int vv,int uu,int nn) :v(vv),u(uu),n(nn) { }
		bool operator<(const Idx &rhs) const { return v==rhs.v?u==rhs.u?n<rhs.n:u<rhs.u:v<rhs.v; }
		bool operator==(const Idx &rhs) const { return v==rhs.v&&u==rhs.u&&n==rhs.n; }
	};

	struct SortByPos {
		SortByPos(const vector<Vec3f> &v) :verts(v) { }
		bool operator()(const Idx &a,const Idx &b) { return verts[a.v].x<verts[b.v].x; }
		const vector<Vec3f> &verts;
	};

	bool Same(const Vec3f &a,const Vec3f &b) {
	//	Vec3f epsilon=VMax(VAbs(a),VAbs(b))*0.0001f;
		Vec3f epsilon(0.00001f,0.00001f,0.00001f);
		return Abs(a.x-b.x)<epsilon.x&&Abs(a.y-b.y)<epsilon.y&&Abs(a.z-b.z)<epsilon.z;
	}
}

void BaseScene::Object::Join(const Object &rhs) {
	int lastTris=tris.size(),lastV=verts.size(),lastU=uvs.size(),lastN=normals.size();
	tris.resize(tris.size()+rhs.tris.size());
	for(int n=0;n<rhs.tris.size();n++) {
		IndexedTri &tri=tris[lastTris+n];
		tri=rhs.tris[n];
		for(int k=0;k<3;k++) {
			tri.v[k]+=lastV;
			tri.vt[k]+=tri.vt[k]>=0?lastU:0;
			tri.vn[k]+=tri.vn[k]>=0?lastV:0;
		}
	}
	verts.resize(lastV+rhs.verts.size());
	uvs.resize(lastU+rhs.uvs.size());
	normals.resize(lastN+rhs.normals.size());
	std::copy(rhs.verts.begin(),rhs.verts.end(),verts.begin()+lastV);
	std::copy(rhs.uvs.begin(),rhs.uvs.end(),uvs.begin()+lastU);
	std::copy(rhs.normals.begin(),rhs.normals.end(),normals.begin()+lastN);
	bbox=BBox(&verts[0],verts.size(),trans);
	optBBox=OptBBox(bbox,Inverse(trans));
}

CompactTris BaseScene::Object::ToCompactTris() const {
	CompactTris out;
	vector<Idx> inds;
	inds.reserve(tris.size() * 3);

	Vec2f defaultUv(0.0f,0.0f);
	Vec3f defaultNrm(0.0f,0.0f,0.0f);

	for(size_t n = 0; n < tris.size(); n++) {
		const IndexedTri &tri = tris[n];
		for(int k = 0; k < 3; k++)
			inds.push_back(Idx(tri.v[k], tri.vt[k], tri.vn[k]));
	}

	{
		std::sort(inds.begin(), inds.end());
		vector<Idx>::iterator end=std::unique(inds.begin(),inds.end());
		inds.resize(end-inds.begin());
	}

	out.verts.resize(inds.size());
	out.normals.resize(inds.size());
	out.tris.resize(tris.size());

//	std::sort(inds.begin(),inds.end(),SortByPos(verts));

	for(int n=0;n<inds.size();n++) {
		out.verts[n] = verts[inds[n].v];
		out.normals[n] = inds[n].n == -1? defaultNrm : normals[inds[n].n];
	}

	for(int n = 0; n < inds.size(); n++)
		inds[n].dstIdx = n;
	std::sort(inds.begin(),inds.end());

	for(int n=0;n<tris.size();n++) {
		const IndexedTri &src=tris[n];
		CompactTris::TriIdx &dst = out.tris[n];

		u32 idx[3];
		for(int k=0;k<3;k++)
			idx[k] = std::lower_bound(inds.begin(), inds.end(), Idx(src.v[k],src.vt[k],src.vn[k]))
				- inds.begin();

		dst.v1 = inds[idx[0]].dstIdx;
		dst.v2 = inds[idx[1]].dstIdx;
		dst.v3 = inds[idx[2]].dstIdx;
		bool flatNrm=	Same(out.normals[dst.v1], out.normals[dst.v2]) &&
						Same(out.normals[dst.v1], out.normals[dst.v3]);
		dst.mat = (src.matId & 0x7fffffff) + (flatNrm? 0x80000000 : 0);
	}
	
	return out;
}

TriangleVector BaseScene::Object::ToTriangleVector() const {
	TriangleVector out;
	vector<Idx> inds;
	inds.reserve(tris.size()*3);

	Vec2f defaultUv(0.0f,0.0f);
	Vec3f defaultNrm(0.0f,0.0f,0.0f);

	for(int n=0;n<tris.size();n++) {
		const IndexedTri &tri=tris[n];
		for(int k=0;k<3;k++) inds.push_back(Idx(tri.v[k],tri.vt[k],tri.vn[k]));
	}

	{
		std::sort(inds.begin(),inds.end());
		vector<Idx>::iterator end=std::unique(inds.begin(),inds.end());
		inds.resize(end-inds.begin());
	}

	out.verts.resize(inds.size());
	out.tris.resize(tris.size());

//	std::sort(inds.begin(),inds.end(),SortByPos(verts));

	for(int n=0;n<inds.size();n++) {
		TriangleVector::Vert &vert=out.verts[n];
		vert.pos=verts[inds[n].v];
		vert.uv=inds[n].u==-1?defaultUv:uvs[inds[n].u];
		vert.nrm=inds[n].n==-1?defaultNrm:normals[inds[n].n];
	}

	for(int n=0;n<inds.size();n++) inds[n].dstIdx=n;
	std::sort(inds.begin(),inds.end());
	for(int n=0;n<tris.size();n++) {
		const IndexedTri &src=tris[n];
		TriangleVector::TriIdx &dst=out.tris[n];

		u32 idx[3];
		for(int k=0;k<3;k++)
			idx[k]=std::lower_bound(inds.begin(),inds.end(),Idx(src.v[k],src.vt[k],src.vn[k]))-inds.begin();

		dst.v1=inds[idx[0]].dstIdx;
		dst.v2=inds[idx[1]].dstIdx;
		dst.v3=inds[idx[2]].dstIdx;
		bool flatNrm=	Same(out.verts[dst.v1].nrm,out.verts[dst.v2].nrm)&&
						Same(out.verts[dst.v1].nrm,out.verts[dst.v3].nrm);
		dst.mat=(src.matId&0x7fffffff)+(flatNrm?0x80000000:0);
	}

	out.triAccels.resize(out.tris.size());
	for(int n=0;n<out.tris.size();n++) {
		const TriangleVector::TriIdx &idx=out.tris[n];
		out.triAccels[n]=TriAccel(out.verts[idx.v1].pos,out.verts[idx.v2].pos,out.verts[idx.v3].pos);
	}
	
	return out;
}

ShTriVector BaseScene::Object::ToShTriVector() const {
	ShTriVector out;
	for(int t=0;t<tris.size();t++)
		out.push_back(GetTriangle(t));
	return out;
}
void BaseScene::GenNormals() {
	for(int n=0;n<objects.size();n++)
		objects[n].GenNormals();
}

Vec3f BaseScene::Center() const {
	if(!objects.size()) return Vec3f(0,0,0);
	Vec3f out=objects[0].Center();
	for(int n=1;n<objects.size();n++)
		out+=objects[n].Center();
	return out/float(objects.size());
}

void BaseScene::Object::FindOptimalTrans() {
	{
		Matrix<Vec4f> min=Identity<>();
		float minSum=1.0f/0.0f;
		
		enum { dx=4,dy=4,dz=4 };
		vector<Vec3f> normals(tris.size());
		for(int n=0;n<tris.size();n++) normals[n]=GetTriangle(n).fnrm;
		
		for(int x=0;x<dx;x++) for(int y=0;y<dy;y++) for(int z=0;z<dz;z++) {
			float ax=dx==1?0.0f:constant::pi*0.5f*float(x)/float(dx-1);
			float ay=dy==1?0.0f:constant::pi*0.5f*float(y)/float(dy-1);
			float az=dz==1?0.0f:constant::pi*0.5f*float(z)/float(dz-1);
			
			trans=Rotate(ax,ay,az);
			
			Vec3f planes[3];
			planes[0]=Vec3f(trans.x.x,trans.x.y,trans.x.z);
			planes[1]=Vec3f(trans.y.x,trans.y.y,trans.y.z);
			planes[2]=Vec3f(trans.z.x,trans.z.y,trans.z.z);
			
			float sum=0.0f;
			for(int n=0;n<tris.size();n++) {
				const Vec3f &nrm=normals[n];
				float m=1.0f/0.0f;
				m=Min(m,Min(planes[0]|nrm,-planes[0]|nrm));
				m=Min(m,Min(planes[1]|nrm,-planes[1]|nrm));
				m=Min(m,Min(planes[2]|nrm,-planes[2]|nrm));
				sum+=m;
			}
			if(sum<minSum) { minSum=sum; min=trans; }
		}
		trans=min;
	}


//	Vec3f center(0.0f,0.0f,0.0f);
//	for(int n=0;n<verts.size();n++) center+=verts[n];
//	center/=float(verts.size());

//	trans.w+=Vec4f(center.x,center.y,center.z,0.0f);

	Matrix<Vec4f> inv=Inverse(trans);
	for(int n=0;n<verts.size();n++) verts[n]=inv*verts[n];
	for(int n=0;n<normals.size();n++) normals[n]=inv&normals[n];
}

void BaseScene::Object::GenNormals() {
	for(int n=0;n<tris.size();n++) {
		IndexedTri &tri=tris[n];
		Vec3f nrm=(verts[tri.v[1]]-verts[tri.v[0]])^(verts[tri.v[2]]-verts[tri.v[0]]);
		nrm*=RSqrt(nrm|nrm);
		if(tri.vn[0]<0||tri.vn[1]<0||tri.vn[2]<0) {
			if(tri.vn[0] < 0) tri.vn[0]=normals.size();
			if(tri.vn[1] < 0) tri.vn[1]=normals.size();
			if(tri.vn[2] < 0) tri.vn[2]=normals.size();
			normals.push_back(nrm);
		}
	}
}

Vec3f BaseScene::Object::Center() const {
	Vec3f out(0,0,0);
	if(verts.size()) {
		for(int n=0;n<verts.size();n++) out+=verts[n];
		out/=float(verts.size());
	}
	return out;
}
