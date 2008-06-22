
#include "triangle.h"

	struct SortTris {
		bool operator()(const Triangle &a,const Triangle &b) const {
			Vec3p min1=a.BoundMin(),min2=b.BoundMin();
			return min1.x<min2.x;
		}
	};

	template <class AccStruct>
	TScene<AccStruct>::TScene(const char *modelFile) :tree(TriVector()),lightsEnabled(1) {
		vector<Object,AlignedAllocator<Object> > objects;
		LoadModel(string(modelFile),objects,shadingData,20.0f,10000000);

	//	AddSoftLight(Vec3f(-2,8.0f,0.9f),Vec3f(800,805,805),Vec3f(40,40,40),1,1,1);
	//	AddSoftLight(Vec3f(-100,-100,0),Vec3f(0,0,20000),Vec3f(1,1,1),1,1,1);
		AddSoftLight(Vec3f(-4000,-3550,-4000),Vec3f(50.0f,50.0f,10.0f)*1000000.0f,Vec3f(30,30,30),1,1,1);
	//	AddSoftLight(Vec3f(-100,-250,0),Vec3f(400000,300000,0),Vec3f(30,30,30),1,1,1);
		
		tree=AccStruct(objects);
	}

	template <class AccStruct>
	void TScene<AccStruct>::Animate() {
		for(int n=0;n<lights.size();n+=1)
			for(int k=n;k<n+1;k++) {
				Vec3f tmp; Convert(lights[k].pos,tmp);
				tmp=RotateY((0.05f+(n/1)*0.01f)*0.2f)*(tmp-Vec3f(0,0,50))+Vec3f(0,0,50);
				Convert(tmp,lights[k].pos);
			}
	}

	template <class AccStruct>
	void TScene<AccStruct>::AddLight(Vec3f pos,Vec3f col) {
		lights.push_back(Light(pos,col));
	}

	template <class AccStruct>
	void TScene<AccStruct>::AddSoftLight(Vec3f pos,Vec3f col,Vec3f dens,int dx,int dy,int dz) {
		assert(dx>0&&dy>0&&dz>0);
		col/=float(dx*dy*dz);

		for(int x=0;x<dx;x++) for(int y=0;y<dy;y++) for(int z=0;z<dz;z++)
			lights.push_back(Light(pos+dens*Vec3f(x/float(dx)-0.5f,y/float(dy)-0.5f,z/float(dz)-0.5f),col));
	}

