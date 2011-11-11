#ifndef RTRACER_VRENDER_H
#define RTRACER_VRENDER_H

#include "rtbase.h"
#include "bounding_box.h"
#include "dicom.h"

class Camera;

void Load3dTexture(const DICOM&);
void RenderVolume(const Camera &cam, float aspectRatio, int resolution);

#endif

