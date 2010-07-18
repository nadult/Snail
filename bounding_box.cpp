#include "bounding_box.h"
#include "ray_group.h"

BBox::BBox(const Vec3f *verts,uint count) {
	if(count==0) {
		min=max=Vec3f(0,0,0);
		return;
	}
	
	min=max=verts[0];
	for(int n=1;n<count;n++) {
		Vec3f vert=verts[n];
		min=VMin(min,vert);
		max=VMax(max,vert);
	}
}

BBox::BBox(const Vec3f *verts,uint count,const Matrix<Vec4f> &trans) {
	if(count==0) {
		min=max=Vec3f(0,0,0);
		return;
	}
	
	min=max=trans*verts[0];
	for(int n=1;n<count;n++) {
		Vec3f vert=trans*verts[n];
		min=VMin(min,vert);
		max=VMax(max,vert);
	}
}


const BBox &BBox::operator*=(const Matrix<Vec4f> &m) {
	Vec3f v[8];
	v[0].x=v[2].x=v[4].x=v[6].x=min.x; v[1].x=v[3].x=v[5].x=v[7].x=max.x;
	v[0].y=v[1].y=v[4].y=v[5].y=min.y; v[3].y=v[2].y=v[6].y=v[7].y=max.y;
	v[0].z=v[1].z=v[2].z=v[3].z=min.z; v[4].z=v[5].z=v[6].z=v[7].z=max.z;

	const Vec3f &tv=v[0];
	min.x=max.x=m.x.x*tv.x+m.y.x*tv.y+m.z.x*tv.z;
	min.y=max.y=m.x.y*tv.x+m.y.y*tv.y+m.z.y*tv.z;
	min.z=max.z=m.x.z*tv.x+m.y.z*tv.y+m.z.z*tv.z;

	for(int n=1;n<8;n++) {
		const Vec3f &tv=v[n];
		float tx=m.x.x*tv.x+m.y.x*tv.y+m.z.x*tv.z;
		float ty=m.x.y*tv.x+m.y.y*tv.y+m.z.y*tv.z;
		float tz=m.x.z*tv.x+m.y.z*tv.y+m.z.z*tv.z;
		if(tx<min.x) min.x=tx; else if(tx>max.x) max.x=tx;
		if(ty<min.y) min.y=ty; else if(ty>max.y) max.y=ty;
		if(tz<min.z) min.z=tz; else if(tz>max.z) max.z=tz;
	}

	min.x+=m.w.x; max.x+=m.w.x;
	min.y+=m.w.y; max.y+=m.w.y;
	min.z+=m.w.z; max.z+=m.w.z;
		
	return *this;
}

template <bool sharedOrigin, bool hasMask>
bool BBox::Test(Context<sharedOrigin, hasMask> &context, int &firstActive, int &lastActive) const {
	bool ret = 0;

	Vec3f tmin, tmax;
	if(sharedOrigin) {
		tmin = min - ExtractN(context.Origin(0), 0);
		tmax = max - ExtractN(context.Origin(0), 0);
	}

	for(int q = firstActive; q <= lastActive; q++) {
		Vec3q idir = context.IDir(q);
		Vec3q origin = context.Origin(q);

		floatq l1 = idir.x * (sharedOrigin? tmin.x : min.x - origin.x);
		floatq l2 = idir.x * (sharedOrigin? tmax.x : max.x - origin.x);
		floatq lmin = Min(l1, l2);
		floatq lmax = Max(l1, l2);

		l1 = idir.y * (sharedOrigin? tmin.y : min.y - origin.y);
		l2 = idir.y * (sharedOrigin? tmax.y : max.y - origin.y);
		lmin = Max(Min(l1, l2), lmin);
		lmax = Min(Max(l1, l2), lmax);

		l1 = idir.z * (sharedOrigin? tmin.z : min.z - origin.z);
		l2 = idir.z * (sharedOrigin? tmax.z : max.z - origin.z);
		lmin = Max(Min(l1, l2), lmin);
		lmax = Min(Max(l1, l2), lmax);

		if(ForAny( lmax >= floatq(0.0f) && lmin <= Min(lmax, context.Distance(q)) )) {
			firstActive = q;
			ret = 1;
			break;
		}
	}
	for(int q = lastActive; q >= firstActive; q--) {
		Vec3q idir = context.IDir(q);
		Vec3q origin = context.Origin(q);

		floatq l1 = idir.x * (sharedOrigin? tmin.x : min.x - origin.x);
		floatq l2 = idir.x * (sharedOrigin? tmax.x : max.x - origin.x);
		floatq lmin = Min(l1, l2);
		floatq lmax = Max(l1, l2);

		l1 = idir.y * (sharedOrigin? tmin.y : min.y - origin.y);
		l2 = idir.y * (sharedOrigin? tmax.y : max.y - origin.y);
		lmin = Max(Min(l1, l2), lmin);
		lmax = Min(Max(l1, l2), lmax);

		l1 = idir.z * (sharedOrigin? tmin.z : min.z - origin.z);
		l2 = idir.z * (sharedOrigin? tmax.z : max.z - origin.z);
		lmin = Max(Min(l1, l2), lmin);
		lmax = Min(Max(l1, l2), lmax);

		if(ForAny( lmax >= floatq(0.0f) && lmin <= Min(lmax, context.Distance(q)) )) {
			lastActive = q;
			ret = 1;
			break;
		}
	}

	return ret;
}

bool BBox::Test(ShadowContext &context, int &firstActive, int &lastActive) const {
	bool ret = 0;

	Vec3f tmin = min - ExtractN(context.Origin(0), 0);
	Vec3f tmax = max - ExtractN(context.Origin(0), 0);

	for(int q = firstActive; q <= lastActive; q++) {
		Vec3q idir = context.IDir(q);

		floatq l1 = idir.x * tmin.x;
		floatq l2 = idir.x * tmax.x;
		floatq lmin = Min(l1, l2);
		floatq lmax = Max(l1, l2);

		l1 = idir.y * tmin.y;
		l2 = idir.y * tmax.y;
		lmin = Max(Min(l1, l2), lmin);
		lmax = Min(Max(l1, l2), lmax);

		l1 = idir.z * tmin.z;
		l2 = idir.z * tmax.z;
		lmin = Max(Min(l1, l2), lmin);
		lmax = Min(Max(l1, l2), lmax);

		if(ForAny( lmax >= 0.0f && lmin <= Min(lmax, context.Distance(q)))) {
			firstActive = q;
			ret = 1;
			break;
		}
	}
	for(int q = lastActive; q >= firstActive; q--) {
		Vec3q idir = context.IDir(q);

		floatq l1 = idir.x * tmin.x;
		floatq l2 = idir.x * tmax.x;
		floatq lmin = Min(l1, l2);
		floatq lmax = Max(l1, l2);

		l1 = idir.y * tmin.y;
		l2 = idir.y * tmax.y;
		lmin = Max(Min(l1, l2), lmin);
		lmax = Min(Max(l1, l2), lmax);

		l1 = idir.z * tmin.z;
		l2 = idir.z * tmax.z;
		lmin = Max(Min(l1, l2), lmin);
		lmax = Min(Max(l1, l2), lmax);

		if(ForAny( lmax >= 0.0f && lmin <= Min(lmax, context.Distance(q)))) {
			lastActive = q;
			ret = 1;
			break;
		}
	}

	return ret;
}

template bool BBox::Test<0, 0>(Context<0, 0>&, int&, int&) const;
template bool BBox::Test<0, 1>(Context<0, 1>&, int&, int&) const;
template bool BBox::Test<1, 0>(Context<1, 0>&, int&, int&) const;
template bool BBox::Test<1, 1>(Context<1, 1>&, int&, int&) const;


bool BBox::TestInterval(const RayInterval &i) const {
	float lmin, lmax;

	float l1, l2, l3, l4;

	l1 = i.minIDir.x * (min.x - i.maxOrigin.x);
	l2 = i.maxIDir.x * (min.x - i.maxOrigin.x);
	l3 = i.minIDir.x * (max.x - i.minOrigin.x);
	l4 = i.maxIDir.x * (max.x - i.minOrigin.x);
	
	lmin = Min(Min(l1, l2), Min(l3, l4));
	lmax = Max(Max(l1, l2), Max(l3, l4));

	l1 = i.minIDir.y * (min.y - i.maxOrigin.y);
	l2 = i.maxIDir.y * (min.y - i.maxOrigin.y);
	l3 = i.minIDir.y * (max.y - i.minOrigin.y);
	l4 = i.maxIDir.y * (max.y - i.minOrigin.y);
	lmin = Max(lmin, Min(Min(l1, l2), Min(l3, l4)));
	lmax = Min(lmax, Max(Max(l1, l2), Max(l3, l4)));

	l1 = i.minIDir.z * (min.z - i.maxOrigin.z);
	l2 = i.maxIDir.z * (min.z - i.maxOrigin.z);
	l3 = i.minIDir.z * (max.z - i.minOrigin.z);
	l4 = i.maxIDir.z * (max.z - i.minOrigin.z);
	lmin = Max(lmin, Min(Min(l1, l2), Min(l3, l4)));
	lmax = Min(lmax, Max(Max(l1, l2), Max(l3, l4)));

	return lmax >= 0.0f && lmin <= lmax;
}

void GaussPointsFit(int iQuantity, const Vec3f* akPoint,Vec3f& rkCenter, Vec3f akAxis[3], float afExtent[3]);

void TGaussPointsFit(int count,const Vec3f* points,Vec3f& center, Vec3f axis[3], float extent[3]) {
	GaussPointsFit(count,points,center,axis,extent);

	Vec3f kDiff = points[0] - center;
	float fY0Min = kDiff|axis[0], fY0Max = fY0Min;
	float fY1Min = kDiff|axis[1], fY1Max = fY1Min;
	float fY2Min = kDiff|axis[2], fY2Max = fY2Min;

	for (int i = 1; i < count; i++) {
		kDiff = points[i] - center;

		float fY0=kDiff|axis[0];
		if(fY0<fY0Min ) fY0Min=fY0;
		else if(fY0>fY0Max) fY0Max=fY0;

		float fY1=kDiff|axis[1];
		if(fY1<fY1Min) fY1Min=fY1;
		else if(fY1>fY1Max) fY1Max=fY1;

		float fY2=kDiff|axis[2];
		if(fY2<fY2Min) fY2Min=fY2;
		else if(fY2>fY2Max) fY2Max=fY2;
	}

	center += (axis[0]*(fY0Min+fY0Max)+axis[1]*(fY1Min+fY1Max)+axis[2]*(fY2Min+fY2Max)) *0.5f;

	extent[0] = 0.5f*(fY0Max - fY0Min);
	extent[1] = 0.5f*(fY1Max - fY1Min);
	extent[2] = 0.5f*(fY2Max - fY2Min);
}

OptBBox::OptBBox(const BBox &b,const Matrix<Vec4f> &m) :box(b),trans(m),invTrans(Inverse(m)) {
}
OptBBox::OptBBox(const BBox &b,const Matrix<Vec4f> &m,const Matrix<Vec4f> &inv) :box(b),trans(m),invTrans(inv) {
}

OptBBox::OptBBox(const Vec3f *verts,uint count) {
	Vec3f center,axis[3]; float ext[3];
	TGaussPointsFit(count,verts,center,axis,ext);
	
	box=BBox(-Vec3f(ext[0],ext[1],ext[2]),Vec3f(ext[0],ext[1],ext[2]));
	trans.x=Vec4f(axis[0].x,axis[0].y,axis[0].z,0.0f);
	trans.y=Vec4f(axis[1].x,axis[1].y,axis[1].z,0.0f);
	trans.z=Vec4f(axis[2].x,axis[2].y,axis[2].z,0.0f);
	trans.w=Vec4f(center.x,center.y,center.z,1.0f);
	invTrans=Inverse(trans);
	
/*{
		Matrix<Vec4f> min,tr;
		BBox minBBox;
		float minSurf=1.0f/0.0f;
		
		for(int n=0;n<32;n++) {
			float ang=ConstPI<float>()*0.5f*float(n)/31.0f;
			tr=RotateY(ang);

			BBox trBBox;
			trBBox.min=trBBox.max=tr*verts[0];
			for(int n=1;n<verts.size();n++) {
				Vec3f vert=tr*verts[n];
				trBBox.min=VMin(trBBox.min,vert);
				trBBox.max=VMax(trBBox.max,vert);
			}
			
			Vec3f size=trBBox.Size();
			float surf=size.x*size.y+size.x*size.z+size.y*size.z;
			if(surf<minSurf) { minSurf=surf; minBBox=trBBox; min=tr; }
		}
		optBBoxTrans=min;
		optBBox=minBBox;
	}*/	
}

OptBBox::operator BBox() const {
	return box*trans;
}	

