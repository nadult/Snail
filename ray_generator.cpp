#include "ray_generator.h"

RayGenerator::RayGenerator(int level,int w,int h,float pd)
	:invW(1.0f/float(w)),invH(1.0f/float(h)),planeDist(pd),tLevel(level) {
	Vec3f add[4]={
		Vec3f(0.0f-w*0.5f,0.0f-h*0.5f,0.0f),
		Vec3f(1.0f-w*0.5f,0.0f-h*0.5f,0.0f),
		Vec3f(0.0f-w*0.5f,1.0f-h*0.5f,0.0f),
		Vec3f(1.0f-w*0.5f,1.0f-h*0.5f,0.0f) };
	Convert(add,addVec);
	invW *= float(w)/float(h);
}

void RayGenerator::Generate(int pw,int ph,int x,int y,Vec3q *out) {
	Generate(tLevel,pw,ph,x,y,out);
}

void RayGenerator::Decompose(const Vec3q *in,Vec3q *out)
{
	const int nQuads=1<<(tLevel*2);

	if(tLevel>=1) {
		for(int n=0;n<nQuads;n+=2) {
			__m128 tmp;
			tmp				=_mm_shuffle(0+(2<<2)+(1<<4)+(3<<6),_mm_unpacklo_ps(in[n+0].X().m,in[n+1].X().m));
			out[n+1].X().m	=_mm_shuffle(0+(2<<2)+(1<<4)+(3<<6),_mm_unpackhi_ps(in[n+0].X().m,in[n+1].X().m));
			out[n+0].X().m=tmp;

			tmp				=_mm_shuffle(0+(2<<2)+(1<<4)+(3<<6),_mm_unpacklo_ps(in[n+0].Y().m,in[n+1].Y().m));
			out[n+1].Y().m	=_mm_shuffle(0+(2<<2)+(1<<4)+(3<<6),_mm_unpackhi_ps(in[n+0].Y().m,in[n+1].Y().m));
			out[n+0].Y().m=tmp;

			tmp				=_mm_shuffle(0+(2<<2)+(1<<4)+(3<<6),_mm_unpacklo_ps(in[n+0].Z().m,in[n+1].Z().m));
			out[n+1].Z().m	=_mm_shuffle(0+(2<<2)+(1<<4)+(3<<6),_mm_unpackhi_ps(in[n+0].Z().m,in[n+1].Z().m));
			out[n+0].Z().m=tmp;
		}
	}
	if(tLevel>=2) {
		for(int n=0;n<nQuads;n+=8) {
			Vec3q *p=out+n,tmp;

			{ const Vec3q tmp=p[2]; p[2]=p[1]; p[1]=p[4]; p[4]=tmp; }
			{ const Vec3q tmp=p[3]; p[3]=p[5]; p[5]=p[6]; p[6]=tmp; }
		}
	}
	if(tLevel>=3) {
		for(int n=0;n<nQuads;n+=32) {
			Vec3q *p=out+n;

			for(int k=0;k<2;k++) {
				Vec3q *pk=p+k;
				{ const Vec3q tmp=pk[2]; pk[2]=pk[16]; pk[16]=pk[8]; pk[8]=pk[4]; pk[4]=tmp;  }
				{ const Vec3q tmp=pk[6]; pk[6]=pk[18]; pk[18]=pk[24]; pk[24]=pk[12]; pk[12]=tmp; }
				{ const Vec3q tmp=pk[14]; pk[14]=pk[22]; pk[22]=pk[26]; pk[26]=pk[28]; pk[28]=tmp; }
				{ const Vec3q tmp=pk[10]; pk[10]=pk[20]; pk[20]=tmp; }
			}
		}
	}
	if(tLevel>=4) {
		for(int n=0;n<nQuads;n+=128) {
			Vec3q *p=out+n;

			for(int k=0;k<4;k++) {
				Vec3q *pk=p+k;
#define CYCLE(A,B,C,D,E)	{ const Vec3q tmp=pk[A]; pk[A]=pk[B]; pk[B]=pk[C]; pk[C]=pk[D]; pk[D]=pk[E]; pk[E]=tmp;  }
				CYCLE(4,64,32,16,8)
				CYCLE(12,68,96,48,24)
				CYCLE(20,72,36,80,40)
				CYCLE(28,76,100,112,56)
				CYCLE(44,84,104,52,88)
				CYCLE(60,92,108,116,120)
#undef CYCLE
			}
		}

	}
}

void RayGenerator::Generate(int level,int pw,int ph,int x,int y,Vec3q *out) {
	if(level==0) {
		Vec3q coords; Broadcast(Vec3f(float(x),float(y),planeDist),coords);

		coords.X()=(coords.X()+addVec.X())*floatq(invW);
		coords.Y()=(coords.Y()+addVec.Y())*floatq(invH);

		out[0]=coords*RSqrt(coords|coords);
		return;
	}

	int npw=pw>>1,nph=ph>>1,nl=level-1;

	Generate(nl,npw,nph,x,y,out+0);
	Generate(nl,npw,nph,x+npw,y,out+(1<<nl*2));
	Generate(nl,npw,nph,x,y+nph,out+(2<<nl*2));
	Generate(nl,npw,nph,x+npw,y+nph,out+(3<<nl*2));
}
