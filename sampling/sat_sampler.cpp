#include "sampling/sat_sampler.h"

namespace sampling {

	SATSampler::SATSampler(const gfxlib::Texture &tex) {
		if(tex.Width()&(tex.Width()-1)||tex.Height()&(tex.Height()-1))
			ThrowException("Texture width & height must be a power of 2");
		if(tex.GetFormat().GetIdent()!=gfxlib::TI_R8G8B8)
			ThrowException("For now only R8G8B8 textures are supported");

		w=tex.Width(); h=tex.Height();
		wMask=w-1; hMask=h-1;
		wShift=1; while((1<<wShift)<w) wShift++;


		const u8 *data=(u8*)tex.DataPointer();
		int pitch=tex.Pitch();

		vector<Sample,AlignedAllocator<Sample> > hSum(w*h);
		samples.resize(w*(h+1));

		for(int y=0;y<h;y++) {
			hSum[y*w]=Sample(data[2+y*pitch],data[1+y*pitch],data[0+y*pitch]);

			for(int x=1;x<w;x++) {
				const u8 *src=data+x*3+y*pitch;
				hSum[x+y*w]=hSum[x-1+y*w]+Sample(src[2],src[1],src[0]);
			}
		}

		for(int x=0;x<w;x++) samples[x]=Sample(0,0,0);
		for(int x=0;x<w;x++) {
			samples[x+w]=hSum[x];
			for(int y=1;y<h;y++)
				samples[x+(y+1)*w]=samples[x+y*w]+hSum[x+y*w];
		}

		const Sample &last=Get(w-1,h-1);
		avg=Vec3f(last.R(),last.G(),last.B())*(1.0f/float(w*h*255));
	}

	INLINE SATSampler::Sample SATSampler::ComputeRect(uint ax,uint ay,uint bx,uint by) const {
		Sample out;
		out=Get(bx,by)-Get(bx,ay-1);
		if(ax) out+=Get(ax-1,ay-1)-Get(ax-1,by);
		return out;
	}

	Vec3f SATSampler::operator()(const Vec2f &uv,const Vec2f &size) const {
		if(size.x>=0.5f||size.y>=0.5f) return avg;
		Vec2f a=uv+size*0.5f,b=uv+size*0.5f;

		a*=Vec2f(w,h); b*=Vec2f(w,h);
		uint ax=a.x,ay=a.y;
		uint bx=b.x,by=b.y;
		ax&=wMask; ay&=hMask;
		bx&=wMask; by&=hMask;

		uint count;
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
				count=(bx-ax+1)*(by+h+1-ay);
				sum=ComputeRect(ax,0,bx,by)+ComputeRect(ax,ay,bx,h-1);
			}
			else {
				count=(by-ay+1)*(bx-ax+1);
				sum=ComputeRect(ax,ay,bx,by);
			}
		}
		return Vec3f(sum.R(),sum.G(),sum.B())*(1.0f/(255.0f*float(count)));
	}

	INLINE void SATSampler::ComputeRect(i32x4 ax,i32x4 ay,i32x4 bx,i32x4 by,Sample out[4]) const {
		for(int k=0;k<4;k++) {
			out[k]=Get(bx[k],by[k])-Get(bx[k],ay[k]-1);
			if(ax[k]) out[k]+=Get(ax[k]-1,ay[k]-1)-Get(ax[k]-1,by[k]);
		}
	}

	Vec3q SATSampler::operator()(const Vec2q &uv,const Vec2q &diff) const {
		f32x4b fullMask=diff.x>=0.5f||diff.x>= 0.5f;
		if(ForAll(fullMask)) return Vec3q(avg.x,avg.y,avg.z);

		Vec2q tDiff=diff*floatq(0.5f);
		Vec2q a=(uv-tDiff),b=(uv+tDiff);
		a*=Vec2q(floatq(w),floatq(h));
		b*=Vec2q(floatq(w),floatq(h));

		i32x4 ax(a.x),ay(a.y);
		i32x4 bx(b.x),by(b.y);
		ax&=wMask; ay&=hMask;
		bx&=wMask; by&=hMask;

		f32x4 count;
		Sample sum[4];
		i32x4 one(1);

		if(ForAll(ax<=bx&&ay<=by)) {
			count=f32x4(by-ay+one)*f32x4(bx-ax+one);
			ComputeRect(ax,ay,bx,by,sum);
		}
		else for(int k=0;k<4;k++) {
			if(ax[k]>bx[k]) {
				if(ay[k]>by[k]) {
					count[k]=(bx[k]+1)*(by[k]+1)+(w-ax[k])*(h-ay[k]);
					sum[k]=ComputeRect(0,0,bx[k],by[k])+ComputeRect(ax[k],ay[k],w-1,h-1);
				}
				else {
					count[k]=(bx[k]+1+w-ax[k])*(by[k]-ay[k]+1);
					sum[k]=ComputeRect(0,ay[k],bx[k],by[k])+ComputeRect(ax[k],ay[k],w-1,by[k]);
				}
			}
			else {
				if(ay[k]>by[k]) {
					count[k]=(bx[k]-ax[k]+1)*(by[k]+h+1-ay[k]);
					sum[k]=ComputeRect(ax[k],0,bx[k],by[k])+ComputeRect(ax[k],ay[k],bx[k],h-1);
				}
				else {
					count[k]=(by[k]-ay[k]+1)*(bx[k]-ax[k]+1);
					sum[k]=ComputeRect(ax[k],ay[k],bx[k],by[k]);
				}
			}
		}

		Vec3q out;
		out.x[0]=sum[0].R(); out.y[0]=sum[0].G(); out.z[0]=sum[0].B();
		out.x[1]=sum[1].R(); out.y[1]=sum[1].G(); out.z[1]=sum[1].B();
		out.x[2]=sum[2].R(); out.y[2]=sum[2].G(); out.z[2]=sum[2].B();
		out.x[3]=sum[3].R(); out.y[3]=sum[3].G(); out.z[3]=sum[3].B();

		return Condition(fullMask,Vec3q(avg.x,avg.y,avg.z),out*Inv(count*255.0f));
	}

}
