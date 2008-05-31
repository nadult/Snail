
#include "triangle.h"
#include "loader.h"

	struct SortTris {
		bool operator()(const Triangle &a,const Triangle &b) const {
			Vec3p min1=a.BoundMin(),min2=b.BoundMin();
			return min1.x<min2.x;
		}
	};

	template <class AccStruct>
	TScene<AccStruct>::TScene(const char *modelFile) :tree(vector<Object>()),lightsEnabled(1) {
		vector<Object> objects;
//		objects.push_back(Sphere(Vec3f(-2,0,5)*5.0f,4.04f*5));
//		objects.push_back(Sphere(Vec3f(3,2,10)*5.0f,3*5));
//		objects.push_back(Sphere(Vec3f(0,0,-2.0)*5.0f,0.3f*5));
//		objects.push_back(Sphere(Vec3f(4.1,1.5f,5)*5.0f,1*5));
//		objects.push_back(Sphere(Vec3f(4.1,0.5f,-3.5f)*5.0f,1*5)); 

//		srand(1235);
//		for(int n=0;n<2000;n++) {
//			Vec3f pos(FRand()*38.0f-20.0f,FRand()*38.0f-20.0f,FRand()*38.0f-20.0f); pos*=8.0f;
//			for(int n=0;n<20;n++)
//			objects.push_back(
//				Sphere(Vec3f(FRand()*7.80f-4.0f,FRand()*7.80f-4.0f,FRand()*7.80f-4.0f)+pos,
//							FRand()*1.50f));
//		}

		{
			vector<Triangle> tris;
			LoadWavefrontObj(modelFile,tris,20.0f);
			objects.resize(tris.size());
			for(uint n=0;n<objects.size();n++)
				objects[n]=tris[n];
		}

	//	AddSoftLight(Vec3f(-2,8.0f,0.9f),Vec3f(800,805,805),Vec3f(40,40,40),1,1,1);
	//	AddSoftLight(Vec3f(-100,-100,0),Vec3f(0,0,20000),Vec3f(1,1,1),1,1,1);
		AddSoftLight(Vec3f(100,-180,0),Vec3f(0,20000,5000),Vec3f(1,1,1),1,1,1);
		AddLight(Vec3f(0,-150,0),Vec3f(15000,20000,0));
		
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

