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
//		if(IsNan(l1) || IsNan(l2))
//			printf("FUCKUP X!\n");

		floatq lmin = Min(l1, l2);
		floatq lmax = Max(l1, l2);

		l1 = idir.y * (sharedOrigin? tmin.y : min.y - origin.y);
		l2 = idir.y * (sharedOrigin? tmax.y : max.y - origin.y);
//		if(IsNan(l1) || IsNan(l2))
//			printf("FUCKUP Y!\n");

		lmin = Max(lmin, Min(l1, l2));
		lmax = Min(lmax, Max(l1, l2));

		l1 = idir.z * (sharedOrigin? tmin.z : min.z - origin.z);
		l2 = idir.z * (sharedOrigin? tmax.z : max.z - origin.z);
//		if(IsNan(l1) || IsNan(l2))
//			printf("FUCKUP Z!\n");

		lmin = Max(lmin, Min(l1, l2));
		lmax = Min(lmax, Max(l1, l2));
		
//		if(IsNan(lmin) || IsNan(lmax))
//			printf("FUCKUP!\n");


		f32x4b mask = lmax < floatq(0.0f) || lmin > Min(lmax, context.Distance(q));
//		if(hasMask) mask = mask && context.SSEMask(q);

		if(!ForAll(mask)) {
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
		lmin = Max(lmin, Min(l1, l2));
		lmax = Min(lmax, Max(l1, l2));

		l1 = idir.z * (sharedOrigin? tmin.z : min.z - origin.z);
		l2 = idir.z * (sharedOrigin? tmax.z : max.z - origin.z);
		lmin = Max(lmin, Min(l1, l2));
		lmax = Min(lmax, Max(l1, l2));

		f32x4b mask = lmax < floatq(0.0f) || lmin > Min(lmax, context.Distance(q));
//		if(hasMask) mask = mask && context.SSEMask(q);
		
		if(!ForAll(mask)) {
			lastActive = q;
			ret = 1;
			break;
		}
	}

	return ret;
}

bool BBox::Test(ShadowContext &context, int &firstActive, int &lastActive) const {
	bool ret = 0;

	Vec3f tmin(min - ExtractN(context.Origin(0), 0));
	Vec3f tmax(max - ExtractN(context.Origin(0), 0));

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

bool BBox::TestCornerRays(const CornerRays &crays) const {
	floatq l1 = crays.idir.x * (min.x - crays.origin.x);
	floatq l2 = crays.idir.x * (max.x - crays.origin.x);
	floatq lmin = Min(l1, l2);
	floatq lmax = Max(l1, l2);

	l1 = crays.idir.y * (min.y - crays.origin.y);
	l2 = crays.idir.y * (max.y - crays.origin.y);
	lmin = Max(Min(l1, l2), lmin);
	lmax = Min(Max(l1, l2), lmax);

	l1 = crays.idir.z * (min.z - crays.origin.z);
	l2 = crays.idir.z * (max.z - crays.origin.z);
	lmin = Max(Min(l1, l2), lmin);
	lmax = Min(Max(l1, l2), lmax);

	//TODO: WRONG!
	return ForAny(lmax >= 0.0f && lmin <= lmax);
}
	
unsigned BBox::WideTest(const Vec3f *origins, const Vec3f *idirs, const u16 *__restrict indices,
					float *maxDist, unsigned count, u16 *__restrict newIndices) const
{
	unsigned newCount = 0;

	floatq minx(min.x), miny(min.y), minz(min.z);
	floatq maxx(max.x), maxy(max.y), maxz(max.z);

	for(unsigned n = 0; n < (count & ~3); n += 4) {
		u16 idx[4] = {indices[n + 0], indices[n + 1], indices[n + 2], indices[n + 3] };

		Vec3q orig; Convert(origins[idx[0]], origins[idx[1]], origins[idx[2]], origins[idx[3]], orig);
		Vec3q idir; Convert(idirs[idx[0]], idirs[idx[1]], idirs[idx[2]], idirs[idx[3]], idir);
		floatq dist(maxDist[idx[0]], maxDist[idx[1]], maxDist[idx[2]], maxDist[idx[3]]); 
		
		floatq l1, l2;
		
		l1 = idir.x * (minx - orig.x);
		l2 = idir.x * (maxx - orig.x);
		floatq lmin = Min(l1, l2);
		floatq lmax = Max(l1, l2);

		l1 = idir.y * (miny - orig.y);
		l2 = idir.y * (maxy - orig.y);
		lmin = Max(Min(l1, l2), lmin);
		lmax = Min(Max(l1, l2), lmax);

		l1 = idir.z * (minz - orig.z);
		l2 = idir.z * (maxz - orig.z);
		lmin = Max(Min(l1, l2), lmin);
		lmax = Min(Max(l1, l2), lmax);

		int result = ForWhich(lmax >= 0.0f && lmin <= Min(lmax, dist));

		if(result & 1) newIndices[newCount++] = idx[0];
		if(result & 2) newIndices[newCount++] = idx[1];
		if(result & 4) newIndices[newCount++] = idx[2];
		if(result & 8) newIndices[newCount++] = idx[3];
	}

	for(unsigned n = count & ~3; n < count; n++) {
		u16 idx = indices[n];

		Vec3f orig = origins[idx];
		Vec3f idir = idirs[idx];
		
		float l1, l2;
		
		l1 = idir.x * (float(min.x) - orig.x);
		l2 = idir.x * (float(max.x) - orig.x);
		float lmin = Min(l1, l2);
		float lmax = Max(l1, l2);

		l1 = idir.y * (float(min.y) - orig.y);
		l2 = idir.y * (float(max.y) - orig.y);
		lmin = Max(Min(l1, l2), lmin);
		lmax = Min(Max(l1, l2), lmax);

		l1 = idir.z * (float(min.z) - orig.z);
		l2 = idir.z * (float(max.z) - orig.z);
		lmin = Max(Min(l1, l2), lmin);
		lmax = Min(Max(l1, l2), lmax);

		if(lmax >= 0.0f && lmin <= Min(lmax, maxDist[idx]))
			newIndices[newCount++] = idx;
	}

	return newCount;
}


/*
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
}*/

