#include "camera.h"
#include "rtbase.h"
#include "tree_stats.h"

template <class AccStruct> class Scene;

struct Options {
	Options(bool refl,bool rdtsc) :reflections(refl),rdtscShader(rdtsc) { }
	Options() { reflections=rdtscShader=0; }

	bool reflections,rdtscShader;
};

template <class AccStruct>
TreeStats<1> Render(const Scene<AccStruct> &scene,const Camera &camera,Image &image,const Options options,uint tasks);

