#ifndef RTRACER_DBVH_H
#define RTRACER_DBVH_H

#include "ray_group.h"
#include "tree_stats.h"
#include "bvh/tree.h"

struct ObjectInstance {
	const BBox GetBBox() const { return bbox; }
	float Center(int axis) const {
		return ((&bbox.min.x)[axis] + (&bbox.max.x)[axis]) * 0.5f;
	}

	bool TestCornerRays(const CornerRays &crays) const {
		return 1;
	}

	bool TestInterval(const RayInterval&) const {
		return 1;
	}

	const Vec3q TransformVec(const Vec3q &v) const {
		return Vec3q(	v.x * rotation[0].x + v.y * rotation[0].y + v.z * rotation[0].z,
						v.x * rotation[1].x + v.y * rotation[1].y + v.z * rotation[1].z,
						v.x * rotation[2].x + v.y * rotation[2].y + v.z * rotation[2].z);
	}
	
	const Vec3q TransformPoint(const Vec3q &p) const {
		return Vec3q(	p.x * rotation[0].x + p.y * rotation[0].y + p.z * rotation[0].z + floatq(translation.x),
						p.x * rotation[1].x + p.y * rotation[1].y + p.z * rotation[1].z + floatq(translation.y),
						p.x * rotation[2].x + p.y * rotation[2].y + p.z * rotation[2].z + floatq(translation.z));
	}
	
	const Vec3q ITransformVec(const Vec3q &v) const {
		return Vec3q(	v.x * rotation[0].x + v.y * rotation[1].x + v.z * rotation[2].x,
						v.x * rotation[0].y + v.y * rotation[1].y + v.z * rotation[2].y,
						v.x * rotation[0].z + v.y * rotation[1].z + v.z * rotation[2].z);
	}
	
	const Vec3q ITransformPoint(Vec3q p) const {
		p -= Vec3q(translation);
		return Vec3q(	p.x * rotation[0].x + p.y * rotation[1].x + p.z * rotation[2].x,
						p.x * rotation[0].y + p.y * rotation[1].y + p.z * rotation[2].y,
						p.x * rotation[0].z + p.y * rotation[1].z + p.z * rotation[2].z);
	}

	void CollidePrimary(Context<0, 0> &c, int idx, int firstActive, int lastActive) const {
		int count = lastActive - firstActive + 1;
		if(!count)
			return;

		Vec3q origins[count], dirs[count], idirs[count];
		for(int n = 0; n < count; n++)
			dirs[n] = ITransformVec(c.Dir(firstActive + n));
		for(int n = 0; n < count; n++)
			idirs[n] = SafeInv(dirs[n]);
		for(int n = 0; n < count; n++)
			origins[n] = ITransformPoint(c.Origin(firstActive + n));
		i32x4 objects[count], temp[count];
		for(int n = 0; n < count; n++)
			objects[n] = i32x4(~0);

		Context<0, 0> newContext(RayGroup<0, 0>(origins, dirs, idirs, count),
				c.distance + firstActive, objects, 0, c.barycentric, c.stats);

		// TODO: zwykle traverse bez corner raysow
		tree->TraversePrimary(newContext);

		for(int n = 0; n < count; n++) {
			i32x4b mask = objects[n] != i32x4(~0);
			c.Object(n + firstActive) = Condition(mask, i32x4(idx), c.Object(n + firstActive));
			c.Element(n + firstActive) = Condition(mask, objects[n], c.Element(n + firstActive));
		}
	}

	void CollidePrimary(Context<0, 1> &c, int idx, int firstActive, int lastActive) const {
		int count = lastActive - firstActive + 1;
		if(!count)
			return;

		Vec3q origins[count], dirs[count], idirs[count];
		for(int n = 0; n < count; n++)
			dirs[n] = ITransformVec(c.Dir(firstActive + n));
		for(int n = 0; n < count; n++)
			idirs[n] = SafeInv(dirs[n]);
		for(int n = 0; n < count; n++)
			origins[n] = ITransformPoint(c.Origin(firstActive + n));
		i32x4 objects[count], temp[count];
		for(int n = 0; n < count; n++)
			objects[n] = i32x4(~0);

		Context<0, 1> newContext(RayGroup<0, 1>(origins, dirs, idirs, count, c.MaskPtr() + firstActive),
				c.distance + firstActive, objects, 0, c.barycentric, c.stats);

		// TODO: zwykle traverse bez corner raysow
		tree->TraversePrimary(newContext);

		for(int n = 0; n < count; n++) {
			i32x4b mask = objects[n] != i32x4(~0);
			c.Object(n + firstActive) = Condition(mask, i32x4(idx), c.Object(n + firstActive));
			c.Element(n + firstActive) = Condition(mask, objects[n], c.Element(n + firstActive));
		}
	}

	void CollidePrimary(Context<1, 1> &c, int idx, int firstActive, int lastActive) const {
		int count = lastActive - firstActive + 1;
		if(!count)
			return;

		Vec3q dirs[count], idirs[count];
		for(int n = 0; n < count; n++)
			dirs[n] = ITransformVec(c.Dir(firstActive + n));
		for(int n = 0; n < count; n++)
			idirs[n] = SafeInv(dirs[n]);
		i32x4 objects[count], temp[count];
		for(int n = 0; n < count; n++)
			objects[n] = i32x4(~0);
		Vec3q origin = ITransformPoint(c.Origin(0));

		Context<1, 1> newContext(RayGroup<1, 1>(&origin, dirs, idirs, count, c.MaskPtr() + firstActive),
				c.distance + firstActive, objects, 0, c.barycentric, c.stats);

		// TODO: zwykle traverse bez corner raysow
		tree->TraversePrimary(newContext);

		for(int n = 0; n < count; n++) {
			i32x4b mask = objects[n] != i32x4(~0);
			c.Object(n + firstActive) = Condition(mask, i32x4(idx), c.Object(n + firstActive));
			c.Element(n + firstActive) = Condition(mask, objects[n], c.Element(n + firstActive));
		}
	}

	void CollidePrimary(Context<1, 0> &c, int idx, int firstActive, int lastActive) const {
		int count = lastActive - firstActive + 1;
		if(!count)
			return;

		Vec3q dirs[count], idirs[count];
		for(int n = 0; n < count; n++)
			dirs[n] = ITransformVec(c.Dir(firstActive + n));
		for(int n = 0; n < count; n++)
			idirs[n] = SafeInv(dirs[n]);
		i32x4 objects[count], temp[count];
		for(int n = 0; n < count; n++)
			objects[n] = i32x4(~0);
		Vec3q origin = ITransformPoint(c.Origin(0));

		Context<1, 0> newContext(RayGroup<1, 0>(&origin, dirs, idirs, count),
				c.distance + firstActive, objects, 0, c.barycentric, c.stats);

		tree->TraversePrimary(newContext);

		for(int n = 0; n < count; n++) {
			i32x4b mask = objects[n] != i32x4(~0);
			c.Object(n + firstActive) = Condition(mask, i32x4(idx), c.Object(n + firstActive));
			c.Element(n + firstActive) = Condition(mask, objects[n], c.Element(n + firstActive));
		}
	}

	void CollideShadow(ShadowContext &c, int firstActive, int lastActive) const {
		int count = lastActive - firstActive + 1;
		if(!count)
			return;

		Vec3q dirs[count], idirs[count];
		for(int n = 0; n < count; n++)
			dirs[n] = ITransformVec(c.Dir(firstActive + n));
		for(int n = 0; n < count; n++)
			idirs[n] = SafeInv(dirs[n]);
		Vec3q origin = ITransformPoint(c.Origin(0));

		ShadowContext newContext(RayGroup<1, 0>(&origin, dirs, idirs, count), c.distance + firstActive, c.stats);

		tree->TraverseShadow(newContext);
	}
	void ComputeBBox();

	const Vec3f GetNormal(int elem) const {
		ThrowException("Write me!");
		return Vec3f(0, 1, 0);
	}

	Vec3f rotation[3];
	Vec3f translation;
	BVH *tree;
	BBox bbox;
};

class DBVH {
public:
	DBVH(const vector<ObjectInstance>&);
	DBVH() { }
	const BBox GetBBox() const { return nodes[0].bbox; }
	void PrintInfo() const;

	//TODO: moze zrobic max 1 dziecko w lisciu, dzieki czemu
	//mozna by nie przechowywac lisci wogole?

public:
	typedef ObjectInstance CElement;
	typedef ShTriangle SElement;

	enum { isComplex = 1 };
	enum { isctFlags = isct::fObject | isct::fElement | isct::fStats };
	enum { maxDepth = 64 };

	const ShTriangle GetSElement(int elem, int subElem) const {
		ShTriangle out = elements[elem].tree->GetSElement(subElem, 0);
		out.Transform(elements[elem].rotation, elements[elem].translation);
		return out;
	}
	const Vec3f GetNormal(int elem, int subElem) const {
		return elements[elem].GetNormal(subElem);
	}

	template <bool sharedOrigin, bool hasMask>
	void TraversePrimary0(Context<sharedOrigin, hasMask> &c, int firstNode = 0) const;

	void TraverseShadow0(ShadowContext &c, int firstNode = 0) const;
	
	template <bool sharedOrigin, bool hasMask>
	void TraversePrimary(Context<sharedOrigin, hasMask> &c) const;
	
	void TraverseShadow(ShadowContext &c) const;

protected:
	void FindSplit(int nNode, int first, int count, int depth);
	void Construct(const vector<ObjectInstance>&);
	int depth;
	
public:
	struct Node {
		Node() { }
		Node(const BBox &bbox) :bbox(bbox) { }
		bool IsLeaf() const { return subNode & 0x80000000; }

		BBox bbox;
		union { int subNode, first; };
		union { struct { short axis, firstNode; }; int count; };
	};
	
	int GetMaterialId(int idx, int elem) const {
		return elements[elem].tree->GetMaterialId(idx, 0);
	}

	vector<ObjectInstance> elements;
	vector<Node, AlignedAllocator<Node> > nodes;
};

SERIALIZE_AS_POD(DBVH::Node)

#endif
