#ifndef RTRACER_TREE_STATS_H
#define RTRACER_TREE_STATS_H

#include "rtbase.h"

namespace stats {

	enum {
		memPatternEnabled	= 0,
		treeStatsEnabled	= 1,
	};

}

class Image;

// Gives information about what memory chunks are needed
// for given view, lights, etc.
// It slows down computation up to 20% (or even more) so
// it should be disabled in release version
struct MemPattern {
	enum { enabled=stats::memPatternEnabled };

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

// disabled TreeStats are zero-sized, and all update functions do nothing
template <bool enabled_>
class TreeStats
{
public:
	enum { enabled=enabled_&&stats::treeStatsEnabled, dataSize=enabled?10:0 };

	inline TreeStats() { if(enabled) Init(); }
	inline TreeStats(const TreeStats &rhs) { if(enabled) for(int n=0;n<dataSize;n++) data[n]=rhs.data[n]; }
	inline void Init() { if(enabled) for(int n=0;n<dataSize;n++) data[n]=0; }
	inline const TreeStats &operator=(const TreeStats &rhs) {
		if(enabled) { for(int n=0;n<dataSize;n++) data[n]=rhs.data[n]; }
		return *this;
	}

	inline const TreeStats &operator+=(const TreeStats &rhs) {
		if(enabled) {
			for(int n=0;n<dataSize;n++)
				data[n]+=rhs.data[n];
		}
		return *this;
	}

	double GetCoherent() const		{ return enabled?double(data[3])/double(data[3]+data[4]):0.0; }
	double GetBreaking() const		{ return enabled?double(data[5])/double(data[5]+data[6]):0.0; }
	double GetIntersectFail() const	{ return enabled?double(data[8])/double(data[7]+data[8]):0.0; }

	uint GetIntersects() const { return enabled?data[0]:0; }
	uint GetLoopIters() const { return enabled?data[1]:0; }
	uint GetTracedRays() const { return enabled?data[2]:0; }
	uint GetSkips() const { return enabled?data[9]:0; }

	string GenInfo(int resx,int resy,double msRenderTime,double msBuildTime);

	void PrintInfo(int resx,int resy,double msRenderTime,double msBuildTime)
		{ printf("%s\n",GenInfo().c_str()); }

	inline void Intersection(uint val=1) { if(enabled) data[0]+=val; }

	inline void IntersectPass(uint val=1) { if(enabled) { data[0]+=val; data[7]+=val; } }
	inline void IntersectFail(uint val=1) { if(enabled) { data[0]+=val; data[8]+=val; } }

	inline void LoopIteration(uint val=1) { if(enabled) data[1]+=val; }

	inline void Breaking(uint val=1) { if(enabled) data[5]+=val; }
	inline void NotBreaking(uint val=1) { if(enabled) data[6]+=val; }

	inline void Skip(uint val=1) { if(enabled) data[9]+=val; }

	inline void TracingRay(uint val=1) { if(enabled) { data[2]+=val; data[4]+=val; } }
	inline void TracingPacket(uint val=1) { if(enabled) { data[2]+=val; data[3]+=val; } }


private:
	u32 data[dataSize];
	u32 dummy;
	// intersects		0
	// iters			1
	// tracedRays		2
	// coherent			3
	// nonCoherent		4
	// breaking			5
	// notBreaking		6
	// intersectPass	7
	// intersectFail	8
	// skips			9
};

#endif

