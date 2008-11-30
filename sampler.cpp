#include "sampler.h"

Sampler::Sampler(const gfxlib::Texture &t) :tex(t) {
	if(tex.Width()&(tex.Width()-1)||tex.Height()&(tex.Height()-1))
		ThrowException("Texture width & height must be a power of 2");
	if(tex.GetFormat().GetIdent()!=gfxlib::TI_R8G8B8)
		ThrowException("For now only R8G8B8 textures are supported");

	wMask=tex.Width()-1;
	hMask=tex.Height()-1;
	wMul=tex.Width(); hMul=tex.Height();
}

template <class Int,class Vec2>
INLINE Vec2 ClampTexCoord(const Vec2 &coord) {
	Vec2 uv=coord-Vec2(Int(coord.x),Int(coord.y));
	uv.x=Condition(uv.x<0.0f,uv.x+1.0f,uv.x);
	uv.y=Condition(uv.y<0.0f,uv.y+1.0f,uv.y);
	return uv;
}

Vec3f Sampler::operator()(const Vec2f &coord) const {
	Vec2f uv=ClampTexCoord<int>(coord);
	Vec2f pos=uv*Vec2f(wMul,hMul);

	int x1(pos.x),y1(pos.y);
	int x2=x1+1,y2=y1+1;

	float dx=pos.x-x1,dy=pos.y-y1;

	const u8 *data=(u8*)tex.DataPointer();
	int pitch=tex.Pitch();

	x1&=wMask; y1&=hMask;
	x2&=wMask; y2&=hMask;
	x1*=3; x2*=3;
	y1*=pitch; y2*=pitch;

	const u8 *p[4]={data+x1+y1,data+x2+y1,data+x1+y2,data+x2+y2};

	Vec3f c[4]={
		Vec3f(p[0][0],p[0][1],p[0][2]),Vec3f(p[1][0],p[1][1],p[1][2]),
		Vec3f(p[2][0],p[2][1],p[2][2]),Vec3f(p[3][0],p[3][1],p[3][2]) };

	return Lerp(Lerp(c[0],c[1],dx),Lerp(c[2],c[3],dx),dy)*(1.0f/255.0f);
}

Vec3q Sampler::operator()(const Vec2q &coord) const {
	Vec2q uv=ClampTexCoord<i32x4>(coord);
	Vec2q pos=uv*Vec2q(wMul,hMul);

	i32x4 x1(pos.x),y1(pos.y);
	i32x4 x2=x1+i32x4(1),y2=y1+i32x4(1);

	floatq dx=pos.x-floatq(x1),dy=pos.y-floatq(y1);

	const u8 *data=(u8*)tex.DataPointer();
	int pitch=tex.Pitch();

	x1&=i32x4(wMask); y1&=i32x4(hMask);
	x2&=i32x4(wMask); y2&=i32x4(hMask);
	x1=x1+x1+x1; x2=x2+x2+x2;

	y1[0]*=pitch; y1[1]*=pitch; y1[2]*=pitch; y1[3]*=pitch;
	y2[0]*=pitch; y2[1]*=pitch; y2[2]*=pitch; y2[3]*=pitch;

	i32x4 o[4]={x1+y1,x2+y1,x1+y2,x2+y2};

#define R(a,b) data[o[a][b]+0]
#define G(a,b) data[o[a][b]+1]
#define B(a,b) data[o[a][b]+2]

	floatq r[4]; {
		r[0]=floatq(R(0,0),R(0,1),R(0,2),R(0,3));
		r[1]=floatq(R(1,0),R(1,1),R(1,2),R(1,3));
		r[2]=floatq(R(2,0),R(2,1),R(2,2),R(2,3));
		r[3]=floatq(R(3,0),R(3,1),R(3,2),R(3,3));
		r[0]=Lerp(Lerp(r[0],r[1],dx),Lerp(r[2],r[3],dx),dy);
	}
	floatq g[4]; {
		g[0]=floatq(G(0,0),G(0,1),G(0,2),G(0,3));
		g[1]=floatq(G(1,0),G(1,1),G(1,2),G(1,3));
		g[2]=floatq(G(2,0),G(2,1),G(2,2),G(2,3));
		g[3]=floatq(G(3,0),G(3,1),G(3,2),G(3,3));
		g[0]=Lerp(Lerp(g[0],g[1],dx),Lerp(g[2],g[3],dx),dy);
	}
	floatq b[4]; {
		b[0]=floatq(B(0,0),B(0,1),B(0,2),B(0,3));
		b[1]=floatq(B(1,0),B(1,1),B(1,2),B(1,3));
		b[2]=floatq(B(2,0),B(2,1),B(2,2),B(2,3));
		b[3]=floatq(B(3,0),B(3,1),B(3,2),B(3,3));
		b[0]=Lerp(Lerp(b[0],b[1],dx),Lerp(b[2],b[3],dx),dy);
	}

#undef R
#undef G
#undef B

	return Vec3q(r[0],g[0],b[0])*f32x4(1.0f/255.0f);
}

SATSampler::SATSampler(const gfxlib::Texture &tex_) :Sampler(tex_) {
	w=tex.Width(); h=tex.Height();
	invW=1.0f/float(w); invH=1.0f/float(h);
	wShift=1; while((1<<wShift)<w) wShift++;

	const u8 *data=(u8*)tex.DataPointer();
	int pitch=tex.Pitch();

	samples.resize(w*h);
	vector<Sample> hSum(samples.size());

	for(int y=0;y<h;y++) {
		hSum[y*w]=Sample(data[y*pitch],data[1+y*pitch],data[2+y*pitch]);

		for(int x=1;x<w;x++) {
			const u8 *src=data+(x-1)*3+y*pitch;
			hSum[x+y*w]=hSum[x-1+y*w]+Sample(src[0],src[1],src[2]);
		}
	}
	for(int x=0;x<w;x++) {
		samples[x]=hSum[x];
		for(int y=1;y<h;y++)
			samples[x+y*w]=samples[x+(y-1)*w]+hSum[x+y*w];
	}

	const Sample &last=samples.back();
	avg=Vec3f(last.R(),last.G(),last.B())*(1.0f/float(w*h*255));
}

INLINE SATSampler::Sample SATSampler::ComputeRect(uint ax,uint ay,uint bx,uint by) const {
	Sample out;
	ay<<=wShift; by<<=wShift;
	out=samples[bx+by];
	if(ax) out-=samples[ax-1+by];
	if(ay) out-=samples[bx+ay-w];
	if(ax&&ay) out+=samples[ax-1+ay-w];
	return out;
}

Vec3f SATSampler::operator()(const Vec2f &uv,const Vec2f &size) const {
//	if(size.x<=invW&&size.y<=invH) return Sampler::operator()(uv);

	Vec3f out;

	Vec2f a=uv-size,b=uv+size;
	if(b.x-a.x>=1.0f||b.y-a.y>=1.0f) return avg;

	int fax=a.x,fay=a.y,fbx=b.x,fby=b.y;

	a-=Vec2f(fax,fay); b-=Vec2f(fax,fay);	

	a.x=Condition(a.x<0.0f,a.x+1.0f,a.x); a.y=Condition(a.y<0.0f,a.y+1.0f,a.y);
	b.x=Condition(b.x<0.0f,b.x+1.0f,b.x); b.y=Condition(b.y<0.0f,b.y+1.0f,b.y);

	a*=Vec2f(w,h); b*=Vec2f(w,h);
	int ax=a.x+0.5f,ay=a.y+0.5f;
	int bx=b.x+0.5f,by=b.y+0.5f;
	ax&=wMask; ay&=hMask;
	bx&=wMask; by&=hMask;

	int count;
	Sample sum;

	if(ax>bx) {
		if(ay>by) {
			count=(bx+1)*(by+1)+(w-ax)*(h-ay);
			sum=ComputeRect(0,0,bx,by)+ComputeRect(ax,ay,w-1,h-1);
		}
		else {
			count=(bx+1+w-ax)*(by-ay+1);
			sum=ComputeRect(0,ay,bx,by)+ComputeRect(ax,ay,w-1,by);
		}
	}
	else {
		if(ay>by) {
			count=(bx-ax+1)*(by+1+h-ay);
			sum=ComputeRect(ax,0,bx,by)+ComputeRect(ax,ay,bx,h-1);
		}
		else {
			count=(by-ay+1)*(bx-ax+1);
			sum=ComputeRect(ax,ay,bx,by);
		}
	}

	/* int full=(fbx-fax)*(fby-fay);
	if(full) {
		double mul=1.0/( (double(count)+double(full)*double(w*h))*255.0 );

		const Sample &last=samples.back();
		out=Vec3f(
				(double(sum.R())+double(last.R()))*mul,
				(double(sum.G())+double(last.G()))*mul,
				(double(sum.B())+double(last.B()))*mul );
		out=Vec3f(1.0f,0.0f,0.0f);
	}
	else */
		out=Vec3f(sum.R(),sum.G(),sum.B())*(1.0f/(255.0f*float(count)));
	
	return out;
}

Vec3q SATSampler::operator()(const Vec2q &uv,const Vec2q &diff) const {
	Vec3f tout[4]={
		operator()(Vec2f(uv.x[0],uv.y[0]),Vec2f(diff.x[0],diff.y[0])),
		operator()(Vec2f(uv.x[1],uv.y[1]),Vec2f(diff.x[1],diff.y[1])),
		operator()(Vec2f(uv.x[2],uv.y[2]),Vec2f(diff.x[2],diff.y[2])),
		operator()(Vec2f(uv.x[3],uv.y[3]),Vec2f(diff.x[3],diff.y[3])) };
	Vec3q out; Convert(tout,out);
	return out;
}
