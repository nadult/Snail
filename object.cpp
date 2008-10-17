#include "object.h"
#include "bihtree.h"

Object::Object(const BIHTree *tree,const Matrix<Vec4f> &mat) :object(tree),trans(mat),invTrans(Inverse(mat)) {
	box=BBox(object->pMin,object->pMax);
	box*=trans;
}

/*
Triangle Object::tris[Object::MaxObjs];
Sphere Object::spheres[Object::MaxObjs];
int Object::nObjs=0;
*/

