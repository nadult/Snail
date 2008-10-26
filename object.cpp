#include "object.h"
#include "bihtree.h"

namespace {

	bool IntersectBBox(const BBox &box,const Vec3f &orig,const Vec3f &dir,float t1=-1.0f) {
		int sign[3]={dir.x<0,dir.y<0,dir.z<0};
		float invDir[3]={1.0f/dir.x,1.0f/dir.y,1.0f/dir.z};
		float bbox[2][3]={{box.min.x,box.min.y,box.min.z},{box.max.x,box.max.y,box.max.z}};
		
		float tmin = (bbox[sign[0]][0] - orig.x) * invDir[0];
		float tmax = (bbox[1-sign[0]][0] - orig.x) * invDir[0];

		float tymin=(bbox[sign[1]][1]-orig.y) * invDir[1];
		float tymax=(bbox[1-sign[1]][1]-orig.y) * invDir[1];

		if((tmin>tymax)||(tymin>tmax)) return false;
		if(tymin>tmin) tmin=tymin;
		if(tymax<tmax) tmax=tymax;

		float tzmin = (bbox[sign[2]][2] - orig.z)*invDir[2];
		float tzmax = (bbox[1-sign[2]][2] - orig.z)*invDir[2];

		if ( (tmin > tzmax) || (tzmin > tmax) ) return false;
		if (tzmin > tmin) tmin = tzmin;
		if (tzmax < tmax) tmax = tzmax;
		
		return ( (tmin<t1||t1<0.0f) && (tmax > 0.0f) );
	}

	bool IntersectBBox(const BBox &box,const Vec3q &orig,const Vec3q &dir,f32x4 t1=-1.0f) {
		f32x4 invDir[3]={Inv(dir.x),Inv(dir.y),Inv(dir.z)};
		
		f32x4 tmin=invDir[0]*(Condition(dir.x<0.0f,f32x4(box.max.x),f32x4(box.min.x))-orig.x);
		f32x4 tmax=invDir[0]*(Condition(dir.x<0.0f,f32x4(box.min.x),f32x4(box.max.x))-orig.x);

		f32x4 tymin=invDir[1]*(Condition(dir.y<0.0f,f32x4(box.max.y),f32x4(box.min.y))-orig.y);
		f32x4 tymax=invDir[1]*(Condition(dir.y<0.0f,f32x4(box.min.y),f32x4(box.max.y))-orig.y);

		if(ForAll((tmin>tymax)||(tymin>tmax))) return false;
		
		tmin=Max(tmin,tymin);
		tmax=Min(tmax,tymax);

		f32x4 tzmin = invDir[2]*(Condition(dir.z<0.0f,f32x4(box.max.z),f32x4(box.min.z))-orig.z);
		f32x4 tzmax = invDir[2]*(Condition(dir.z<0.0f,f32x4(box.min.z),f32x4(box.max.z))-orig.z);

		if ( ForAll((tmin > tzmax) || (tzmin > tmax)) ) return false;

		tmin=Max(tmin,tzmin);
		tmax=Min(tmax,tzmax);
		
		return ForAll((tmin<t1||t1<0.0f)&&(tmax>0.0f));
	}
}


vector<Object*> gObjects;


void BVH::TraverseMono(const Vec3p &o,const Vec3p &d,Output<otNormal,float,u32> output,int nNode) const {
	const Node &node=nodes[nNode];

	if(!IntersectBBox(node.box,o,d,output.dist[0]))
		return;
		
	Vec3f tOrig,tDir; {
		const Matrix<Vec4f> &m=node.invTrans;
		tOrig.x = m.x.x*o.x+m.y.x*o.y+m.z.x*o.z+m.w.x;
		tOrig.y = m.x.y*o.x+m.y.y*o.y+m.z.y*o.z+m.w.y;
		tOrig.z = m.x.z*o.x+m.y.z*o.y+m.z.z*o.z+m.w.z;
		
		tDir.x = m.x.x*d.x+m.y.x*d.y+m.z.x*d.z;
		tDir.y = m.x.y*d.x+m.y.y*d.y+m.z.y*d.z;
		tDir.z = m.x.z*d.x+m.y.z*d.y+m.z.z*d.z;
	}

	if(!node.count) {
		gObjects[node.subNode]->TraverseMono(tOrig,tDir,output);
		return;
	}
	
	for(int n=0;n<node.count;n++)
		TraverseMono(tOrig,tDir,output,node.subNode+n);
}

void BVH::TraverseQuad(const Vec3q &o,const Vec3q &d,Output<otNormal,f32x4,i32x4> output,int nNode) const {
	const Node &node=nodes[nNode];

	Vec3q tOrig,tDir; {
		const Matrix<Vec4f> &m=node.invTrans;
		tOrig.x = o.x*m.x.x+o.y*m.y.x+o.z*m.z.x+f32x4(m.w.x);
		tOrig.y = o.x*m.x.y+o.y*m.y.y+o.z*m.z.y+f32x4(m.w.y);
		tOrig.z = o.x*m.x.z+o.y*m.y.z+o.z*m.z.z+f32x4(m.w.z);
		
		tDir.x = d.x*m.x.x+d.y*m.y.x+d.z*m.z.x;
		tDir.y = d.x*m.x.y+d.y*m.y.y+d.z*m.z.y;
		tDir.z = d.x*m.x.z+d.y*m.y.z+d.z*m.z.z;
	}

	if(!IntersectBBox(node.box,tOrig,tDir,output.dist[0]))
		return;
		
	output.stats->Skip();

	if(!node.count) {
		gObjects[node.subNode]->TraverseQuad(tOrig,tDir,output);
		return;
	}
	
	for(int n=0;n<node.count;n++)
		TraverseQuad(tOrig,tDir,output,node.subNode+n);
}

void BVH::UpdateBox(int nNode) {
	Node &node=nodes[nNode];
	
	if(node.count) {
		UpdateBox(node.subNode);
		node.box=nodes[node.subNode].box*nodes[node.subNode].trans;
	
		for(int n=1;n<node.count;n++) {
			UpdateBox(node.subNode+n);
			node.box+=nodes[node.subNode+n].box*nodes[node.subNode+n].trans;
		}
	}
	else node.box=gObjects[node.subNode]->GetBBox();
}

BBox BVH::GetBBox() const {
	return nodes[0].box;
}
