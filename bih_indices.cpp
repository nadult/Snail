#include <set>
#include <algorithm>
#include <float.h>
#include "bihtree.h"

namespace {

	int ClipTri(const Vec3f *tri,Vec3f *out,int axis,float pos,bool less) {
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

	int ClipTris(const Vec3f *tris,int count,Vec3f *out,int axis,float pos,float dir) {
		int nOut=0;
		bool less=dir<0.0f;

		for(int n=0;n<count;n++)
			nOut+=ClipTri(tris+n*3,out+nOut*3,axis,pos,less);
		return nOut;
	}

	bool MinimizeTriBound(const Vec3f &p1,const Vec3f &p2,const Vec3f &p3,Vec3f &min,Vec3f &max) {
		Vec3f buf1[3*32],buf2[3*32]; Vec3f *src=buf1,*dst=buf2;
		int count;

		float eps=0.0001f;

		src[0]=p1; src[1]=p2; src[2]=p3; count=1;
		if(max.x-min.x>eps) {
			count=ClipTris(src,count,dst,0,min.x,+1); Swap(src,dst);
			count=ClipTris(src,count,dst,0,max.x,-1); Swap(src,dst);
		}
		if(max.y-min.y>eps) {
			count=ClipTris(src,count,dst,1,min.y,+1); Swap(src,dst);
			count=ClipTris(src,count,dst,1,max.y,-1); Swap(src,dst);
		}
		if(max.z-min.z>eps) {
			count=ClipTris(src,count,dst,2,min.z,+1); Swap(src,dst);
			count=ClipTris(src,count,dst,2,max.z,-1); Swap(src,dst);
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

	template <class Container>
	void Split(const Triangle &tri,const BIHIdx &idx,Container &out) {
		Vec3f size=idx.max-idx.min;
		int axis=size.x>size.y?(size.z>size.x?2:0):(size.z>size.y?2:1);

		Vec3f min1=idx.min,min2=idx.min,max1=idx.max,max2=idx.max;
		Vec3f p1,p2,p3;	Convert(tri.P1(),p1); Convert(tri.P2(),p2); Convert(tri.P3(),p3);
		float split=Lerp((&idx.min.x)[axis],(&idx.max.x)[axis],0.5f);
		(&max1.x)[axis]=(&min2.x)[axis]=split;

		float mult=GetTriMultiplier(tri);

		bool add1=MinimizeTriBound(p1,p2,p3,min1,max1);
		bool add2=MinimizeTriBound(p1,p2,p3,min2,max2);
		
		if(add1) out.insert(BIHIdx(idx.idx,min1,max1,mult));
		if(add2) out.insert(BIHIdx(idx.idx,min2,max2,mult));
	}

	struct SortByIdx { bool operator()(const BIHIdx &a,const BIHIdx &b) const { return a.idx<b.idx; } };
	struct SortBySize { bool operator()(const BIHIdx &a,const BIHIdx &b) const { return a.size>b.size; } };

}

void GenBIHIndices(const vector<Triangle> &tris,vector<BIHIdx> &out,float maxSize,uint maxSplits) {
	std::multiset<BIHIdx,SortBySize> indices;

	for(int n=0;n<tris.size();n++) {
		Vec3p min=tris[n].BoundMin(),max=tris[n].BoundMax();
		indices.insert(BIHIdx(n,Vec3f(min.x,min.y,min.z),Vec3f(max.x,max.y,max.z),GetTriMultiplier(tris[n])));
	}

	for(int s=0;s<maxSplits;s++) {
		BIHIdx idx=*indices.begin();
		if(idx.size<maxSize) break;
		indices.erase(indices.begin());
		Split(tris[idx.idx],idx,indices);
	}
	std::set<BIHIdx,SortBySize>::iterator it=indices.begin();
	out.reserve(indices.size());

	while(it!=indices.end()) out.push_back(*it++);
	std::sort(out.begin(),out.end(),SortByIdx());
}

