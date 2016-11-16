#include "camera.h"
#include "rtbase.h"
#include "tree_stats.h"

namespace fwk { class Texture; }

template <class AccStruct> class Scene;

struct Options {
	Options(bool refl,bool rdtsc) :reflections(refl), rdtscShader(rdtsc) { }
	Options() { reflections = rdtscShader = 0; }

	bool reflections, rdtscShader;
};

template <class AccStruct>
TreeStats Render(const Scene<AccStruct> &scene, const Camera &camera, uint resx, uint resy,
				unsigned char *data, const vector<int> &coords, const vector<int> &offsets,
				const Options options, uint rank, uint threads);

template <class AccStruct>
TreeStats Render(const Scene<AccStruct> &scene, const Camera &camera,
					MipmapTexture &image, const Options options, uint threads);

template <class AccStruct>
TreeStats RenderAndSend(const Scene<AccStruct> &scene, const Camera &camera, uint resx, uint resy,
				unsigned char *data, unsigned char *comprData, const vector<int> &coords,
				const vector<int> &offsets, const Options options, uint rank, uint threads);
