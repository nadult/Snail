#pragma once

#include "rtbase.h"

namespace stats {

	enum {
		memPatternEnabled	= 0,
		treeStatsEnabled	= 1,
	};

}

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
			if(p>=0 && p<res) data[p]+=count;
		}
	}
	void Draw(MipmapTexture &img) const;

private:
	int res; float mul;
	vector<int> data;
};

class TreeStats
{
public:
	enum {
		enabled = stats::treeStatsEnabled,
		dataSize = enabled? 10 : 1,
		timersSize = enabled? 8 : 1
   	};

	inline TreeStats() { if(enabled) Init(); }
	inline TreeStats(const TreeStats &rhs) {
		if(enabled) {
			for(int n = 0; n < dataSize; n++) data[n] = rhs.data[n];
			for(int n = 0; n < timersSize; n++) timers[n] = rhs.timers[n]; 
		}
	}
	inline void Init() {
		if(enabled) {
			for(int n = 0; n < dataSize; n++) data[n] = 0;
			for(int n = 0; n < timersSize; n++) timers[n] = 0;
		}
	}
	inline const TreeStats &operator=(const TreeStats &rhs) {
		if(enabled) {
			for(int n = 0;n < dataSize; n++) data[n]=rhs.data[n];
			for(int n = 0; n < timersSize; n++) timers[n] = rhs.timers[n];
		}
		return *this;
	}

	inline const TreeStats &operator+=(const TreeStats &rhs) {
		if(enabled) {
			for(int n = 0; n < dataSize; n++)
				data[n] += rhs.data[n];
			for(int n = 0; n < (int)sizeof(timers) / sizeof(int); n++)
				timers[n] += rhs.timers[n];
		}
		return *this;
	}

	double GetBreaking() const		{ return enabled?double(data[5])/double(data[5]+data[6]):0.0; }
	double GetIntersectFail() const	{ return enabled?double(data[8])/double(data[7]+data[8]):0.0; }

	uint GetIntersects() const { return enabled?data[0] : 0; }
	uint GetLoopIters() const { return enabled?data[1] : 0; }
	uint GetSkips() const { return enabled?data[9] : 0; }
	uint GetRays() const { return enabled?data[2] : 0; }

	const string GenInfo(int resx,int resy,double msRenderTime,double msBuildTime) const;
	void PrintInfo(int resx,int resy,double msRenderTime,double msBuildTime) const;

	inline void Intersection(uint val=1) { if(enabled) data[0]+=val; }

	inline void IntersectPass(uint val=1) { if(enabled) { data[0]+=val; data[7]+=val; } }
	inline void IntersectFail(uint val=1) { if(enabled) { data[0]+=val; data[8]+=val; } }

	inline void LoopIteration(uint val=1) { if(enabled) data[1]+=val; }

	inline void Breaking(uint val=1) { if(enabled) data[5]+=val; }
	inline void NotBreaking(uint val=1) { if(enabled) data[6]+=val; }

	inline void Skip(uint val=1) { if(enabled) data[9]+=val; }

	inline void TracingRays(uint val=1) { if(enabled) { data[2]+=val; } }

	int timers[timersSize];
private:
	u32 data[dataSize];
	// intersects		0
	// iters			1
	// tracedRays		2
	// 
	// 
	// breaking			5
	// notBreaking		6
	// intersectPass	7
	// intersectFail	8
	// skips			9
};
