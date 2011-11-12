#ifndef RTRACER_VTREE_H
#define RTRACER_VTREE_H

#include "rtbase.h"
#include "bounding_box.h"
#include "volume_data.h"

struct Block {
	enum { size = 4, size3 = size * size * size };
	u16 data[size3];
};

struct Node {
	bool IsLeaf() const { return axis == 3; }

	u16 min, max;
	union {
		struct { i32 leftChild, rightChild; };
		union { i32 child[2]; };
	};
	i32 axis;
	float split;
};

struct VTree {
	vector<Block> blocks;
	vector<Node> nodes;
	vector<BBox> boxes;

	int width, height, depth;
	int startNode;
	BBox treeBBox;

	void BuildTree(int w, int h, int d, int firstNode, int count);

	u16 Trace(const Vec3f &origin, const Vec3f &dir, int node, float rmin, float rmax, u16 min) const;
	u16 Trace(const Vec3f &origin, const Vec3f &dir, u16 min) const;
	BBox GetBBox() const { return treeBBox; }

	VTree(const VolumeData &dicom);
	VTree() = default;
};

class Camera;

void RenderTree(gfxlib::Texture &image, const VTree &tree, const Camera &camera);

#endif
