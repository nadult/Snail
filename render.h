#include "camera.h"
#include "rtbase.h"
#include "tree_stats.h"
#include "gfxlib_texture.h"

template <class AccStruct> class Scene;

struct Options {
	Options(bool refl,bool rdtsc) :reflections(refl), rdtscShader(rdtsc) { }
	Options() { reflections = rdtscShader = 0; }

	bool reflections, rdtscShader;
};

template <class AccStruct>
TreeStats<1> Render(const Scene<AccStruct> &scene, const Camera &camera,
					gfxlib::Texture &image, const Options options, uint threads);

template <class AccStruct>
TreeStats<1> Render(const Scene<AccStruct> &scene, const Camera &camera,
					gfxlib::Texture &image, uint rank, uint nRanks, uint strapHeight,
					const Options options, uint threads);

struct CompressedPart {
	struct Info {
		int x, y, w, h, size;
	} info;
	vector<char> data;
};

int CompressParts(const gfxlib::Texture &image, uint rank, uint nRanks, uint strapHeight,
				uint threads, vector<CompressedPart> &buffers);
