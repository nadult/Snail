#ifndef RTRACER_BASE_SCENE_H
#define RTRACER_BASE_SCENE_H

#include "rtbase.h"


class BaseScene {
public:
	void LoadWavefrontObj(const char *fileName);

	class Vertex {
	public:
		Vec3f pos,nrm;
		Vec2f uv;
	};
	
	class Triangle {
	public:
		Vertex verts[3];
		Vec3f nrm;
	};
	
	class Object {
	public:
		vector<u32> indices;
		vector<Vertex> verts;
		
		Triangle GetTriangle(uint n) const;
		
		inline uint TrisCount() const { return indices.size()/3; }
		inline uint VertsCount() const { return verts.size(); }
		
		string name;
	};
	
	vector<Object> objects;
};

#endif
