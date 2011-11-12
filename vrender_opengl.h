#ifndef RTRACER_VRENDER_H
#define RTRACER_VRENDER_H

#include "rtbase.h"
#include "bounding_box.h"
#include "volume_data.h"

class Camera;

void Load3dTexture(const VolumeData&);
void RenderVolume(const Camera &cam, float aspectRatio, int resolution);

#endif

