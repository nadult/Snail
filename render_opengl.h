#ifndef RTRACER_RENDER_OPENGL_H
#define RTRACER_RENDER_OPENGL_H

#include "rtbase.h"
#include "scene.h"

class BVH;
class Camera;

class OGLRenderer
{
public:
	OGLRenderer(const Scene<BVH> &bvh);
	~OGLRenderer();

	void Draw(const Camera &cam, float fov, float aspect) const;
	void operator=(const OGLRenderer&) = delete;
	OGLRenderer(const OGLRenderer&) = delete;

private:
	void InitShaders();

	unsigned triCount;
	unsigned posBuffer, uvBuffer, nrmBuffer;
	unsigned program;
	bool useNormals, useUvs;
	BBox bbox;
};

#endif
