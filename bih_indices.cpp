#include <set>
#include <algorithm>
#include <float.h>
#include "bihtree.h"

namespace {

	template <int axis,bool less>
	int ClipTri(const Vec3f *tri,Vec3f *out,float pos) {
		int okSide[3];
		okSide[0]=((&tri[0].x)[axis]<pos)==less;
		okSide[1]=((&tri[1].x)[axis]<pos)==less;
		okSide[2]=((&tri[2].x)[axis]<pos)==less;
		int count=okSide[0]+okSide[1]+okSide[2];

		if(count==0) {
			return 0;
		}
		if(count==3) {
			out[0]=tri[0]; out[1]=tri[1]; out[2]=tri[2];
			return 1;
		}

		bool test=count==1?true:false;

		Vec3f a,b,c,ba,ca;
		if(okSide[0]==test) { a=tri[0]; b=tri[1]; c=tri[2]; }
		if(okSide[1]==test) { a=tri[1]; b=tri[2]; c=tri[0]; }
		if(okSide[2]==test) { a=tri[2]; b=tri[0]; c=tri[1]; }
		ba=b-a; ca=c-a;
		float t1=(pos-(&a.x)[axis])/(&ba.x)[axis];
		float t2=(pos-(&a.x)[axis])/(&ca.x)[axis];
		
		if(count==1) {
			out[0]=a; out[1]=a+ba*t1; out[2]=a+ca*t2;
			return 1;
		}
		else { //count==2
			ba=a+ba*t1; ca=a+ca*t2;
			out[0]=ba; out[1]=b; out[2]=c;
			out[3]=ba; out[4]=c; out[5]=ca;
			return 2;
		}
	}

	template <int axis,bool less>
	int ClipTris(const Vec3f *tris,int count,Vec3f *out,float pos) {
		int nOut=0;

		for(int n=0;n<count;n++)
			nOut+=ClipTri<axis,less>(tris+n*3,out+nOut*3,pos);
		return nOut;
	}

	bool MinimizeTriBound(const Vec3f &p1,const Vec3f &p2,const Vec3f &p3,Vec3f &min,Vec3f &max) {
		Vec3f buf1[3*32],buf2[3*32]; Vec3f *src=buf1,*dst=buf2;
		int count;

		float eps=0.0001f;

		src[0]=p1; src[1]=p2; src[2]=p3; count=1;
		if(max.x-min.x>eps) {
			count=ClipTris<0,0>(src,count,dst,min.x); Swap(src,dst);
			count=ClipTris<0,1>(src,count,dst,max.x); Swap(src,dst);
		}
		if(max.y-min.y>eps) {
			count=ClipTris<1,0>(src,count,dst,min.y); Swap(src,dst);
			count=ClipTris<1,1>(src,count,dst,max.y); Swap(src,dst);
		}
		if(max.z-min.z>eps) {
			count=ClipTris<2,0>(src,count,dst,min.z); Swap(src,dst);
			count=ClipTris<2,1>(src,count,dst,max.z); Swap(src,dst);
		}

		if(count) {
			Vec3f tMin=src[0],tMax=src[0];
			count*=3;

			for(int n=1;n<count;n++) {
				tMin=VMin(tMin,src[n]);
				tMax=VMax(tMax,src[n]);
			}
			tMin-=Vec3f(eps,eps,eps);
			tMax+=Vec3f(eps,eps,eps);

			min=VMax(min,tMin);
			max=VMin(max,tMax);
		}

		return count>0;
	}

	// The closer to the worse case, the bigger value
	float GetTriMultiplier(const Triangle &tri) {
		const float min=1.0f,max=4.0f;

		Vec3p worse(0.577350269,0.577350269,0.577350269);
		float out=Lerp(min,max,VAbs(tri.Nrm())|worse);
		return out>=min&&out<=max?out:1.0f;
	}

	struct SortByIdx { bool operator()(const BIHIdx &a,const BIHIdx &b) const { return a.idx<b.idx; } };
	struct SortBySize { bool operator()(const BIHIdx &a,const BIHIdx &b) const { return a.size>b.size; } };

}

void SplitIndices(const Vector<Triangle> &tris,vector<BIHIdx> &inds,int axis,float pos,float maxSize) {
	if(inds.size()<=8) return;
	maxSize*=1.75f;

	for(int n=0,end=inds.size();n<end;n++) {
		BIHIdx &idx=inds[n];
		if(idx.size<maxSize) continue;
		if((&idx.min.x)[axis]>=pos||(&idx.max.x)[axis]<=pos) continue;

		const Triangle &tri=tris[idx.idx];

		Vec3f min1=idx.min,min2=idx.min,max1=idx.max,max2=idx.max;
		Vec3f p1,p2,p3;	Convert(tri.P1(),p1); Convert(tri.P2(),p2); Convert(tri.P3(),p3);
		(&max1.x)[axis]=(&min2.x)[axis]=pos;

		float mult=GetTriMultiplier(tri);

		int add1=MinimizeTriBound(p1,p2,p3,min1,max1);
		int add2=MinimizeTriBound(p1,p2,p3,min2,max2);
		
		if(add1) {
			idx=BIHIdx(idx.idx,min1,max1,mult);
			if(add2) inds.push_back(BIHIdx(idx.idx,min2,max2,mult));
		}
		else if(add2) idx=BIHIdx(idx.idx,min2,max2,mult);
	}
}

