#ifndef RTRACER_BASE_SCENE_H
#define RTRACER_BASE_SCENE_H

#include "rtbase.h"
#include "triangle.h"

class BaseScene {
public:
	void LoadWavefrontObj(const string &fileName);
	TriVector ToTriVector() const;
	BBox GetBBox() const;

	class Triangle {
	public:
		operator ::Triangle() const { return ::Triangle(pos[0],pos[1],pos[2]); }
		
		Vec3f pos[3],nrm[3],fnrm;
		Vec2f uv[3];
	};
	
	struct IndexedTri {
		u32 v[3];
		i32 vt[3],vn[3]; // if <0 then not used
	};
	
	class Object {
	public:
		Object(const vector<Vec3f> &verts,const vector<Vec2f> &uvs,const vector<Vec3f> &normals,const vector<IndexedTri> &tris);
		Object() { }
		
		Triangle GetTriangle(uint n) const;
		
		TriVector ToTriVector() const;
		void FindOptimalTrans();
		
		inline BBox GetBBox() const { return bbox; }
		inline Matrix<Vec4f> GetTrans() const { return trans; }
		
		string name;
		Matrix<Vec4f> trans;
		
	protected:
		vector<IndexedTri> tris;
		vector<Vec3f> verts;
		vector<Vec2f> uvs;
		vector<Vec3f> normals;
		BBox bbox;
		
		void UpdateBox();
		
		friend class BaseScene;
	};
	
	vector<Object> objects;
	BBox bbox;
};

#endif
