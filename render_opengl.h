#pragma once

#include "rtbase.h"
#include "scene.h"
#include "photons.h"

class BVH;
class Camera;

class OGLRenderer
{
public:
	OGLRenderer(const Scene<BVH> &bvh);
	~OGLRenderer();

	void BeginDrawing(const Camera &cam, float fov, float aspect, bool clearColor) const;
	void FinishDrawing() const;

	void Draw() const;
	void DrawPhotons(const vector<Photon> &photons, bool update);
	void operator=(const OGLRenderer&) = delete;
	OGLRenderer(const OGLRenderer&) = delete;

private:
	void InitShaders();

	unsigned triCount;
	unsigned posBuffer, uvBuffer, nrmBuffer;
	unsigned program;
	unsigned photonBuffer, photonColorsBuffer, photonBufferSize;
	bool useNormals, useUvs;
	BBox bbox;
};
