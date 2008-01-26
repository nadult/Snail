#include "ray_group.h"

class PacketIdGenerator
{
	static int base[256];
public:
	static void Init() {
		for(int n=0;n<256;n++) base[n]=1;
	}
	static INLINE int Gen() {
		int threadId=omp_get_thread_num();
		return (threadId<<24)+(base[threadId]++);
	}
	static INLINE int Gen(int last) {
		int threadId=last>>24;
		return (threadId<<24)+(base[threadId]++);
	}
};


class SlowKDNode
{
public:
	SlowKDNode() :type(T_LEAF) { }
	enum Type { T_X=0, T_Y=1, T_Z=2, T_LEAF=3 } type;

	bool notEmpty;
	float pos;
	u32 child; // left: child+0    right: child+1
	vector<u32> objects;
	Vec3f pMin,pMax;
};

class SlowKDTree
{
public:
	enum { MaxLevel=30 };

	SlowKDTree(const vector<Object> &objects);
	void Draw(Image&,Vec3<float>,Vec3<float>,const Camera &cam,u32 node=0) const;

private:
	friend class KDTree;

	void Build(u32 node,u32 level,Vec3<float>,Vec3<float>);
	SSEPVec3 pMin,pMax;
	vector<SlowKDNode> nodes;
	vector<Object> objects;
};

class KDNode
{
public:
	u32 ChildDist() const { return val&0x3fffffff; }
	u32 FirstObject() const { return val&0x3fffffff; }
	u32 NumObjects() const { return num; }

	u32 Axis() const { return (val>>30); }
	bool Leaf() const { return (val>>30)==3; }
	float Pos() const { return pos; }

	void SetLeaf(u32 first,u32 n) { num=n; val=first|(3u<<30); }
	void SetNode(u32 axis,float p,u32 nextAdv) { pos=p; val=nextAdv|(axis<<30); }


private:
	union { float pos; u32 num; };
	u32 val;
};

struct NormalOutput
{
	enum { objectIdsFlag=1 };
	NormalOutput(floatq *distance,u32 *objectIds) :dist(distance),object(objectIds) { }
	NormalOutput(const NormalOutput &all,int n) :dist(all.dist+n),object(all.object+n*4) { }

	floatq *dist;
	u32 *object;
};

struct ShadowOutput
{
	enum { objectIdsFlag=0 };
	ShadowOutput(floatq *distance) :dist(distance) { }
	ShadowOutput(const ShadowOutput &all,int n) :dist(all.dist+n) { }

	floatq *dist;
	u32 *object; // dummy
};

class KDStats
{
public:
	KDStats() {
		Init();
	}
	void Update(const KDStats &local) {
		colTests+=local.colTests;
		iters+=local.iters;
		runs+=local.runs;
		tracedRays+=local.tracedRays;
		coherent+=local.coherent;
		nonCoherent+=local.nonCoherent;
		breaking+=local.breaking;
		notBreaking+=local.notBreaking;
	}
	void Init() {
		memset(this,0,sizeof(KDStats));
	}
	double CoherentPercentage() const {
		return 100.0*coherent/double(coherent+nonCoherent);
	}
	double BreakingPercentage() const {
		return 100.0*breaking/double(breaking+notBreaking);
	}
	void PrintInfo(int resx,int resy,double cycles,double msRenderTime) {
			double raysPerSec=double(tracedRays)*(1000.0/msRenderTime);
			double nPixels=double(resx*resy);

			printf("isct*4/pix:%.4f iter/pix:%.4f MCycles/frame:%.2f\tRays/sec:%.0f\tCoherency:%.2f%% br:%.2f%%\n",
				double(colTests)/nPixels,double(iters)/nPixels,
				cycles,raysPerSec,
				CoherentPercentage(),BreakingPercentage());
	}

	u32 colTests,iters,runs,tracedRays;
	u32 coherent,nonCoherent;

	u32 breaking,notBreaking;
};

class KDTree
{
public:
	enum { MaxLevel=SlowKDTree::MaxLevel };

	KDTree(const SlowKDTree&);
	~KDTree();

	template <class Output,class Vec,class base>
	void FullTraverse(const Vec &rOrigin,const Vec &rDir,const base &maxD,const Output&) const;

	template <class Output,class Vec,class base>
	void Traverse(int packetId,const Vec &rOrigin,const Vec &rDir,const base &maxD,const Output &out) const;

	template <class Output,class Group>
	void TraverseFast(int packetId,Group &group,const RaySelector<Group::size>&,const floatq &maxD,const Output &out) const;

	template <class Output,class Group>
	void TraverseOptimized(int packetId,Group &group,const RaySelector<Group::size>&,const floatq &maxD,const Output &out) const;

	bool TestNode(Vec3f min,Vec3f max,int node) const;
	bool Test() const;

	// Clears lastVisit paremeter in objects
	void Prepare() const;

	SSEPVec3 pMin,pMax;
	vector<Object> objects;

private:
	vector<KDNode> nodes;
	vector<u32> objectIds;
	SSEPVec3 *nodeMinMax;

public:
	mutable KDStats stats;
};

#include "kdtraversal.h"
