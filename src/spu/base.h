#pragma once

#include <spu_mfcio.h>
#include "rtbase_math.h"
#include "spu/trace.h"

void Local2Mem(unsigned long long memAddr, volatile void *lsAddr, unsigned int size, int tag = 0);
void Mem2Local(unsigned long long memAddr, volatile void *lsAddr, unsigned int size, int tag = 0);

inline void Local2MemAsync(unsigned long long memAddr, volatile void *lsAddr,
							unsigned int size, int tag = 0) {
	mfc_put(lsAddr, memAddr, size, tag, 0, 0);
}

inline void Mem2LocalAsnyc(unsigned long long memAddr, volatile void *lsAddr,
							unsigned int size, int tag = 0) {
	mfc_get(lsAddr, memAddr, size, tag, 0, 0);
}

inline void DMAWait(int tagmask = 1) {
	mfc_write_tag_mask(tagmask);
	mfc_read_tag_status_all();
}

enum ContextType {
	ctPrimary,
	ctShadow,
	ctSecondary,
};


struct PrimaryContext {
	PrimaryContext() :reflections(0) { }

	enum { type = ctPrimary };

	Vec3q rayDir[NQuads + 3];
	Vec3q rayIDir[NQuads + 3];
	Vec3q rayOrigin;
	Vec2q barycentric[NQuads + 3];
	floatq distance[NQuads + 3];

	Vec3q normals[NQuads + 3];
	Vec3q colors[NQuads + 3];
	i32x4 triIds[NQuads + 3];

	int reflections;
};

struct ShadowContext {
	ShadowContext() { }

	enum { type = ctShadow };

	Vec3q rayDir[NQuads + 3], rayIDir[NQuads + 3], rayOrigin;
	floatq distance[NQuads + 3];
};

struct SecondaryContext {
	enum { type = ctSecondary };

	Vec3q rayDir[NQuads], rayIDir[NQuads], rayOrigin[NQuads];
	Vec2q barycentric[NQuads];
	floatq distance[NQuads];
	
	Vec3q normals[NQuads];
	Vec3q colors[NQuads];
	i32x4 triIds[NQuads];

	int reflections;
};

class RayInterval {
public:
	RayInterval(const Vec3q*, const Vec3q*, const Vec3q, int count);
	RayInterval(const Vec3q*, const Vec3q*, const Vec3q*, int count);
	
	floatq ix, iy, iz;
	Vec3f minIDir, maxIDir, minDir, maxDir;
	Vec3f minOrigin, maxOrigin;
};

template <class T, int tsize, int tag>
struct Cache {
	enum {
		size = tsize,
		mask = tsize - 1,
		tagmask = 1 << tag,
	};
	Cache() :dataPtr(0) { }

	void Init(unsigned long long tdataPtr) {
		dataPtr = tdataPtr;
		for(int n = 0; n < tsize; n++)
			indices[n] = -1;
	}

	void LoadObjects(int first, int count) {
		for(int i = 0; i < count; ) {
			int hash = (first + i) & mask;
			int tcount = size - hash < count - i? size - hash : count - i;
			bool noneed = 1;
			for(int k = 0; k < tcount; k++)
				noneed &= indices[hash + k] == first + i + k;
			if(noneed) {
				i += tcount;
				continue;
			}

			mfc_get(objects + hash, dataPtr + (first + i) * sizeof(T), tcount * sizeof(T), tag, 0, 0);
			mfc_write_tag_mask(tagmask);
			for(int k = 0; k < tcount; k++)
				indices[hash + k] = first + i + k;
			i += tcount;
		}
	}

	void WaitForDMA() {
		mfc_write_tag_mask(tagmask);
		mfc_read_tag_status_all();
	}

	const T& operator[](int idx) {
		int hash = idx & mask;
		if(indices[hash] != idx) {
			Mem2Local(dataPtr + idx * sizeof(T), objects + hash, sizeof(T), tag);
			indices[hash] = idx;
		}
		return objects[hash];
	}
	// UNSAFE; You have to be sure, that the object is loaded
	const T& operator()(int idx) { return objects[idx & mask]; }

	T objects[tsize] ALIGN256;
	unsigned long long dataPtr;
	int indices[tsize];
};
