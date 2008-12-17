#ifndef RTRACER_BASE_SCENE_H
#define RTRACER_BASE_SCENE_H

#include "rtbase.h"
#include "triangle.h"
#include <stdio.h>
#include <map>


class BaseScene {
public:
	void LoadWavefrontObj(const string &fileName);
	void LoadDoom3Proc(const string &fileName);
	void SaveWavefrontObj(const string &fileName) const;
	
	TriangleVector ToTriangleVector() const;
	TriVector ToTriVector() const;
	ShTriVector ToShTriVector() const;

	BBox GetBBox() const;
	
	void Transform(const Matrix<Vec4f> &mat);
	void TransformData(const Matrix<Vec4f> &mat);
	void GenNormals();
	void FlipNormals();

	void Optimize();

	Vec3f Center() const;

	class Triangle {
	public:
		operator ::Triangle() const {
			return ::Triangle(pos[0],pos[1],pos[2]);
		}
		operator ::ShTriangle() const {
			return ::ShTriangle(pos[0],pos[1],pos[2],uv[0],uv[1],uv[2],nrm[0],nrm[1],nrm[2],matId);
		}
		
		Vec3f pos[3],nrm[3],fnrm;
		Vec2f uv[3];
		int matId;
	};
	
	struct IndexedTri {
		i32 v[3];
		i32 vt[3],vn[3]; // if <0 then not used
		i32 matId;
	};
	
	class Object {
	public:
		Object(const vector<Vec3f> &verts,const vector<Vec2f> &uvs,const vector<Vec3f> &normals,const vector<IndexedTri> &tris);
		Object() { }

		// erases bad triangles
		void Repair();

		Triangle GetTriangle(uint n) const;

		TriangleVector ToTriangleVector() const;
		TriVector ToTriVector() const;
		ShTriVector ToShTriVector() const;

		void FlipNormals();
		
		void TransformData(const Matrix<Vec4f> &mat);
		void Transform(const Matrix<Vec4f> &mat);
		void BreakToElements(vector<Object> &output);
		void GenNormals();
		void Optimize();

		Vec3f Center() const;
		
		// transform included
		inline BBox GetBBox() const { return bbox; }
		
		// transform not included
		inline OptBBox GetOptBBox() const { return optBBox; }

		inline Matrix<Vec4f> GetTrans() const { return trans; }
		
		const string &GetName() const { return name; }
		void SetName(const string &str) { name=str; }
		
	protected:
		vector<IndexedTri> tris;
		vector<Vec3f> verts;
		vector<Vec2f> uvs;
		vector<Vec3f> normals;
		
		BBox bbox;
		string name;
		OptBBox optBBox;
		Matrix<Vec4f> trans;
		
		void FindOptimalTrans();
		
		friend class BaseScene;
	};
	
	std::map<string,int> matNames;
	vector<Object> objects;
	BBox bbox;
};

#endif
