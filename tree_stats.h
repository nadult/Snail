#ifndef RTRACER_TREE_STATS_H
#define RTRACER_TREE_STATS_H

#include "rtbase.h"

class Image;

// Gives information about what memory chunks are needed
// for given view, lights, etc.
// It slows down computation up to 20% (or even more) so
// it should be disabled in release version
struct MemPattern {
	enum { enabled=0 };

	void Init(int size,int r=256);
	inline void Touch(int pos,int count=1) {
		if(enabled) {
			int p=float(pos)*mul;
			if(p>=0&&p<res) data[p]+=count;
		}
	}
	void Draw(Image &img) const;

private:
	int res; float mul;
	vector<int> data;
};


class TreeStats
{
public:
	enum { enabled=1 };

	inline TreeStats() { Init(); }
	inline void Update(const TreeStats &local) {
		if(enabled) {
			intersects+=local.intersects;
			iters+=local.iters;
			tracedRays+=local.tracedRays;
			coherent+=local.coherent;
			nonCoherent+=local.nonCoherent;
			breaking+=local.breaking;
			notBreaking+=local.notBreaking;

			intersectPass+=local.intersectPass;
			intersectFail+=local.intersectFail;

			skips+=local.skips;
		}
	}

	inline void Init() {
		if(enabled) for(int n=0;n<sizeof(TreeStats)/sizeof(u32);n++) ((u32*)this)[n]=0;
	}
	double Coherent() const			{ return enabled?coherent/double(coherent+nonCoherent):0; }
	double TBreaking() const		{ return enabled?breaking/double(breaking+notBreaking):0; }
	double TIntersectFail() const	{ return enabled?intersectFail/double(intersectFail+intersectPass):0; }

	uint TracedRays() const { return enabled?tracedRays:0; }
	uint Intersects() const { return enabled?intersects:0; }
	uint LoopIters() const { return enabled?iters:0; }
	uint Skips() const { return enabled?skips:0; }

	void PrintInfo(int resx,int resy,double msRenderTime,double msBuildTime);

	inline void Breaking(uint val=1) { if(enabled) breaking+=val; }
	inline void NotBreaking(uint val=1) { if(enabled) notBreaking+=val; }

	inline void LoopIteration(uint val=1) { if(enabled) iters+=val; }
	inline void Intersection(uint val=1) { if(enabled) intersects+=val; }
	inline void Skip(uint val=1) { if(enabled) skips+=val; }

	inline void IntersectFail(uint val=1) { if(enabled) intersectFail+=val; }
	inline void IntersectPass(uint val=1) { if(enabled) intersectPass+=val; }

	inline void TracingRay(uint val=1) { if(enabled) { tracedRays+=val; nonCoherent+=val; } }
	inline void TracingPacket(uint val=1) { if(enabled) { tracedRays+=val; coherent+=val; } }


//private:
	u32 intersects,iters,tracedRays;
	u32 coherent,nonCoherent;

	u32 breaking,notBreaking;
	u32 intersectPass,intersectFail;
	u32 skips;
};

#endif

