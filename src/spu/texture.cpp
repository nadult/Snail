#include "spu/texture.h"
#include "spu/stats.h"

static Cache<TextureInfo, 16, 4> texInfoCache;

	
struct Line {
	unsigned char bytes[32];
	unsigned char &operator[](int n) { return bytes[n]; }
	unsigned char operator[](int n) const { return bytes[n]; }
};

template <int size>
struct TexelCache {
	Line lines[size] ALIGN256;
	unsigned long long ptrs[size];

	void Clear() {
		for(int n = 0; n < size; n++)
			ptrs[n] = 0;
	}

	unsigned long long AlignPtr(unsigned long long ptr) const {
		return ptr & ~(unsigned long long)(sizeof(Line) - 1);
	}

	const Line &operator[](unsigned long long ptr) {
		int hash = (ptr >> 5) & (size - 1);

		if(EXPECT_NOT_TAKEN(ptrs[hash] != ptr)) {
			Mem2Local(ptr, lines + hash, sizeof(Line));
			ptrs[hash] = ptr;
		}

		return lines[hash];
	}
};

static TexelCache<128> texCache;

void InitTexCache(unsigned long long texInfoPtr) {
	texInfoCache.Init(texInfoPtr);
	texCache.Clear();
}

const Vec3q SampleTexture(int texId, Vec2q uv) {
	BlockTimer timer(timerSampling);

	if(EXPECT_NOT_TAKEN(texId == ~0))
		return Vec3q(1, 1, 1);
	
	const TextureInfo texInfo = texInfoCache[texId];
	if(EXPECT_NOT_TAKEN(texInfo.dataPtr == 0))
		return Vec3q(1, 1, 1);

	uv *= Vec2q(float(texInfo.width - 1), float(texInfo.height - 1));
	i32x4 x4(uv.x), y4(uv.y);
	x4 &= i32x4(texInfo.wMask);
	y4 &= i32x4(texInfo.hMask);

	Vec3q ret;

	for(int k = 0; k < 4; k++) {
		int x = x4[k], y = texInfo.height - 1 - y4[k];

		unsigned long long ptr = texInfo.dataPtr + (x + y * texInfo.width) * 3;
		unsigned char col[3];

		if(EXPECT_TAKEN((ptr & (sizeof(Line) - 1)) < sizeof(Line) - 2)) {
			unsigned long long aptr = texCache.AlignPtr(ptr);
			const Line &line = texCache[aptr];
			int offset = ptr & (sizeof(Line) - 1);
			col[0] = line[offset + 0];
			col[1] = line[offset + 1];
			col[2] = line[offset + 2];
		}
		else {
			for(int i = 0; i < 3; i++) {
				unsigned long long tptr = ptr + i;
				unsigned long long aptr = texCache.AlignPtr(tptr);
				const Line &line = texCache[aptr];
				col[i] = line[tptr & (unsigned long long)(sizeof(Line) - 1)];
			}
		}

		ret.x[k] = float(col[0]);
		ret.y[k] = float(col[1]);
		ret.z[k] = float(col[2]);
	}

	return ret * floatq(1.0f / 255.0f);
}

