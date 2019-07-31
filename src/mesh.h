#include "rtbase.h"
#include "triangle.h"

class MeshAnim;

class Mesh {
public:
	void Load(const string &fileName);
	void Animate(const MeshAnim &anim,float pos);
	void SetMaterial(int id);

	struct Joint { string name; int parent; Vec3f pos; Vec4f rot; };
	struct Vert { Vec2f uv; int weightIdx,weightCount; };
	struct Weight { int jointIdx; float value; Vec3f pos; };
	struct Tri { int idx[3]; };

	vector<Joint> joints;
	vector<Vert> verts;
	vector<Tri> tris;
	vector<Weight> weights;

	TriangleVector triVec;
	int matId;
};

class MeshAnim {
public:
	void Load(const string &fileName);
	void GetFrame(float,Vec3f*,Vec4f*) const;
	void GetBaseFrame(Vec3f*,Vec4f*) const;

	struct Hierarchy { string name; int parent,numComp,frameIdx; };

	vector<Hierarchy> hierarchy;
	vector<BBox> boxes;
	vector<Vec3f> framePos;
	vector<Vec4f> frameRot;
	int frameRate,numJoints,numFrames;
};
