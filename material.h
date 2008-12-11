#ifndef RTRACER_MATERIAL_H
#define RTRACER_MATERIAL_H

#include "rtbase.h"
#include "light.h"

namespace shading {

	class LightSample {
		Vec3q fromLight;
		floatq lightDist;
		f32x4b shadowMask;
	};

	struct Sample {
		enum { maxLightSamples=4 };

		floatq distance;

		Vec3q position;
		Vec3q normal;
		Vec3q reflection;

		Vec2q texCoord;
		Vec2q texDiff;

		f32x4b mask;
		i32x4 matId;

		LightSample lightSamples[maxLightSamples];

		Vec3q color,light;
	};

	template <class Vec,class real>
	Vec ShadeLight(const Vec &lightColor,const real &dot,const real &lightDist) {
		Vec out;
		real mul=Inv(lightDist*lightDist);
	//	real spec=dot*dot; spec=spec*spec;// spec*=spec; spec*=spec;
		out = ( 
				 lightColor*dot
		//		+Vec3q(lightColor.x,0.0f,0.0f)*spec
				)*mul;
		return out;
	}

	template <class Sampler,bool NDotR=true>
	class SimpleMaterial {
	public:
		SimpleMaterial(int tId,const Sampler &tSampler) :sampler(tSampler),id(tId+1) { }
		SimpleMaterial() { }

		template <int size,int flags>
		void PreShade(Sample *__restrict__ samples,const RayGroup<size,flags> &rays,int first,int count) const {
			const i32x4 matId(id);

			for(int q=first,end=first+count;q<end;q++) {
				Sample &s=samples[q];
				const f32x4b mask(s.matId==matId);
				if(!ForAny(mask)) continue;

				Vec3q color=sampler(s.texCoord,s.texDiff);
				if(NDotR) color*=rays.Dir(q)|s.normal;

				s.color=Condition(mask,color,s.color);
				s.light=Condition(mask,Vec3q(0.0f,0.0f,0.0f),s.light);
			}
		}

/*		void LightShade(Sample &s,const Light *lights,int lightsCount,Vec3q rayDir) const {
			for(int k=0;k<lightsCount;k++) {
				const LightSample &ls=s.lightSamples[k];
				const Light &light=lights[k];
				Vec3q lightCol(light.color.x,light.color.y,light.color.z);

				s.light=Condition(ls.shadowMask,s.light,
						s.light+ShadeLight(lightCol,ls.fromLight|rayDir,ls.lightDist));
			}
		}*/

		void PostShade(Sample &s) {
		}


	private:
		Sampler sampler;
		int id;
	};


}

#endif

