#include "object.h"

Triangle Object::tris[Object::MaxObjs];
//Sphere Object::spheres[Object::MaxObjs];
Vec3p Object::bounds[Object::MaxObjs*2];
int Object::nObjs=0;

