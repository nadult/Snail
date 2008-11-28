#include "sampler.h"

Sampler::Sampler(const gfxlib::Texture &t) :tex(t) {
}

Vec3f Sampler::operator()(const Vec2f &uv) const {
	Vec2f pos=uv*Vec2f(float(tex.Width()),float(tex.Height()));
	int x(pos.x),y(pos.y);
	float dx=pos.x-x,dy=pos.y-y;

	assert(tex.GetFormat().GetIdent()==gfxlib::TI_R8G8B8);
	u8 *data=(u8*)tex.DataPointer();
	int pitch=tex.Pitch();

	x=x%tex.Width();
	y=y%tex.Height();

	int x2=(x+1)%tex.Width();
	int y2=(y+1)%tex.Height();

	u8 *p[4]={data+x*3+y*pitch,data+x2*3+y*pitch,data+x*3+y2*pitch,data+x2*3+y2*pitch};

	Vec3f c[4]={
		Vec3f(p[0][2],p[0][1],p[0][0]),Vec3f(p[1][2],p[1][1],p[1][0]),
		Vec3f(p[2][2],p[2][1],p[2][0]),Vec3f(p[3][2],p[3][1],p[3][0]) };

	return Lerp(Lerp(c[0],c[1],dx),Lerp(c[2],c[3],dx),dy)*(1.0f/255.0f);
}

Vec3q Sampler::operator()(const Vec2q &uv) const {
	Vec2q pos=uv*Vec2q(float(tex.Width()),float(tex.Height()));
	i32x4 x(pos.x),y(pos.y);

	assert(tex.GetFormat().GetIdent()==gfxlib::TI_R8G8B8);
	u8 *data=(u8*)tex.DataPointer();
	int pitch=tex.Pitch();

	for(int n=0;n<4;n++) {
		x[n]=x[n]%tex.Width();
		y[n]=y[n]%tex.Height();
	}
//	x%=i32x4(tex.Width()); y%=i32x4(tex.Height());
	i32x4 offset=x+x+x+y*i32x4(pitch);

	u8 *pix[4]={data+offset[0],data+offset[1],data+offset[2],data+offset[3]};

	Vec3q out;
	out.z[0]=pix[0][0]; out.y[0]=pix[0][1]; out.x[0]=pix[0][2];
	out.z[1]=pix[1][0]; out.y[1]=pix[1][1]; out.x[1]=pix[1][2];
	out.z[2]=pix[2][0]; out.y[2]=pix[2][1]; out.x[2]=pix[2][2];
	out.z[3]=pix[3][0]; out.y[3]=pix[3][1]; out.x[3]=pix[3][2];

	return out*f32x4(1.0f/255.0f);
}


