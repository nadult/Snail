#include "base_scene.h"


BaseScene::Triangle BaseScene::Object::GetTriangle(uint n) const {
	Triangle out;
	for(int k=0;k<3;k++) out.verts[k]=verts[n*3+k];
	out.nrm=(out.verts[1].pos-out.verts[0].pos)^(out.verts[2].pos-out.verts[0].pos);
	out.nrm*=RSqrt(out.nrm|out.nrm);
	return out;
}
