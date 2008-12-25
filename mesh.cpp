#include "mesh.h"
#include <iostream>
#include <fstream>

void Mesh::Load(const string &fileName) {
	std::filebuf fb;
	if(!fb.open (fileName.c_str(),std::ios::in)) return;

	std::istream in(&fb);
	joints.clear();
	
	while(!in.eof()) {
		string token; in >> token;

		if(token == "joints") {
			in >> token; assert(token=="{");
			while(true) {
				if(in.eof()) ThrowException("Error while loading mesh from ",fileName);
				in >> token;
				if(token=="}") break;

				Joint newJoint; string p1,p2;
				newJoint.name=token; in >> newJoint.parent;
				newJoint.name = newJoint.name.substr(1,newJoint.name.length()-2);

			   	in >> p1 >> newJoint.pos.x >> newJoint.pos.z >> newJoint.pos.y >> p2; assert(p1=="("&&p2==")");
			   	in >> p1 >> newJoint.rot.x >> newJoint.rot.z >> newJoint.rot.y >> p2; assert(p1=="("&&p2==")");
				newJoint.rot.w = 1.0f-newJoint.rot.x*newJoint.rot.x-newJoint.rot.y*newJoint.rot.y-
										newJoint.rot.z*newJoint.rot.z;
				newJoint.rot.w=newJoint.rot.w<0.0f?0.0f:sqrt(newJoint.rot.w);
				joints.push_back(newJoint);
			}
		}
		else if(token=="mesh") {
			int numVerts=~0,numTris=~0,numWeights=~0;

			in >> token; assert(token=="{");
			while(true) {
				if(in.eof()) ThrowException("Error while loading mesh from ",fileName);
				in >> token;
				if(token=="}") break;

				if(token=="vert") {
					if(numVerts==~0) ThrowException("Error while loading mesh from ",fileName);
					int id; string p1,p2; in >> id;
					Vert &vert=verts[id];
					in >> p1 >> vert.uv.x >> vert.uv.y >> p2 >> vert.weightIdx >> vert.weightCount;
					assert(p1=="("&&p2==")");
				}
				else if(token=="tri") {
					if(numTris==~0) ThrowException("Error while loading mesh from ",fileName);
					int id; in >> id;
					Tri &tri=tris[id];
					in >> tri.idx[0] >> tri.idx[2] >> tri.idx[1];
				}
				else if(token=="weight") {
					if(numWeights==~0) ThrowException("Error while loading mesh from ",fileName);
					int id; string p1,p2; in >> id;
					Weight &weight=weights[id];
					in >> weight.jointIdx >> weight.value;
					in >> p1 >> weight.pos.x >> weight.pos.z >> weight.pos.y >> p2;
					assert(p1=="("&&p2==")");
				}
				else if(token=="numverts"  ) { in >> numVerts;   verts.resize(numVerts); 	 }
				else if(token=="numtris"   ) { in >> numTris;    tris.resize(numTris);   	 }
				else if(token=="numweights") { in >> numWeights; weights.resize(numWeights); }
			}
		}
	}

	triVec.tris.resize(tris.size());
	for(int n=0;n<tris.size();n++) {
		triVec.tris[n].v1=tris[n].idx[0];
		triVec.tris[n].v2=tris[n].idx[1];
		triVec.tris[n].v3=tris[n].idx[2];
	}
	triVec.verts.resize(verts.size());
	for(int n=0;n<verts.size();n++) triVec.verts[n].uv=verts[n].uv;
	triVec.triAccels.resize(tris.size());

	SetMaterial(0);
}

void Mesh::SetMaterial(int id) {
	matId=id;
	for(int n=0;n<tris.size();n++) triVec.tris[n].mat=matId;
}

namespace {

	void Quat2RotMat(const Vec4f &quat,Vec3f *mat) {
		float fTx  = 2.0f*quat.x;
		float fTy  = 2.0f*quat.y;
		float fTz  = 2.0f*quat.z;
		float fTwx = fTx*quat.w;
		float fTwy = fTy*quat.w;
		float fTwz = fTz*quat.w;
		float fTxx = fTx*quat.x;
		float fTxy = fTy*quat.x;
		float fTxz = fTz*quat.x;
		float fTyy = fTy*quat.y;
		float fTyz = fTz*quat.y;
		float fTzz = fTz*quat.z;

		mat[0].x = 1.0f-(fTyy+fTzz);
		mat[0].y = fTxy-fTwz;
		mat[0].z = fTxz+fTwy;
		mat[1].x = fTxy+fTwz;
		mat[1].y = 1.0f-(fTxx+fTzz);
		mat[1].z = fTyz-fTwx;
		mat[2].x = fTxz-fTwy;
		mat[2].y = fTyz+fTwx;
		mat[2].z = 1.0f-(fTxx+fTyy);
	}

	Vec3f Trans(const Vec4f &quat,const Vec3f &pos) {
		Vec3f out;
		Vec3f mat[3];
		Quat2RotMat(quat,mat);
		out.x = pos.x*mat[0].x+pos.y*mat[0].y+pos.z*mat[0].z;
		out.y = pos.x*mat[1].x+pos.y*mat[1].y+pos.z*mat[1].z;
		out.z = pos.x*mat[2].x+pos.y*mat[2].y+pos.z*mat[2].z;
		return out;
	}

	Vec4f MulQuat(const Vec4f &qa,const Vec4f &qb) {
		Vec4f r;
		r.w = (qa.w * qb.w) - (qa.x * qb.x) - (qa.y * qb.y) - (qa.z * qb.z);
		r.x = (qa.x * qb.w) + (qa.w * qb.x) + (qa.y * qb.z) - (qa.z * qb.y);
		r.y = (qa.y * qb.w) + (qa.w * qb.y) + (qa.z * qb.x) - (qa.x * qb.z);
		r.z = (qa.z * qb.w) + (qa.w * qb.z) + (qa.x * qb.y) - (qa.y * qb.x);
		r *= RSqrt(r|r);
		return r;
	}

	Vec4f Slerp(const Vec4f &q1,const Vec4f &q2,float tx) {
		Vec4f out;
		float ang=q1|q2,sign=1.0f,x=tx,t0,t1;

		if(ang<0.0f) { ang=-ang; sign=-1.0f; }
		if(ang<0.999f) {
			float th=acos(ang),tth=th*x,isin=1.0f/Sqrt(1.0f-ang*ang);
			t0=Sin(float(th-tth))*isin;
			t1=Sin(float(tth))*isin;
		}
		else { t0=1.0f-x; t1=x*sign; }

		out=q1*t0+q2*t1;
		out*=RSqrt(out|out);
		return out;
	}

}

void Mesh::Animate(const MeshAnim &anim,float pos) {
	Vec3f aPos[joints.size()];
	Vec4f aRot[joints.size()];
	anim.GetFrame(pos,aPos,aRot);

	for(int n=0;n<verts.size();n++) {
		triVec.verts[n].pos=Vec3f(0.0f,0.0f,0.0f);
		triVec.verts[n].nrm=Vec3f(0.0f,0.0f,0.0f);
	}

	for(int n=0;n<verts.size();n++) {
		const Vert &vert=verts[n];
		Vec3f sum(0.0f,0.0f,0.0f);

		for(int k=0;k<vert.weightCount;k++) {
			const Weight &weight=weights[vert.weightIdx+k];
			Vec3f pos=Trans(aRot[weight.jointIdx],weight.pos);
			sum += (pos+aPos[weight.jointIdx])*weight.value;
		}
		triVec.verts[n].pos=sum;
	}
	for(int n=0;n<tris.size();n++) {
		const Tri &tri=tris[n];
		TriangleVector::Vert 	&v1=triVec.verts[tri.idx[0]],
								&v2=triVec.verts[tri.idx[1]],
								&v3=triVec.verts[tri.idx[2]];
		TriAccel &triAcc=triVec.triAccels[n];
		triAcc=TriAccel(v1.pos,v2.pos,v3.pos);
		v1.nrm+=triAcc.Nrm();
		v2.nrm+=triAcc.Nrm();
		v3.nrm+=triAcc.Nrm();
	}
	for(int n=0;n<verts.size();n++)
		triVec.verts[n].nrm*=RSqrt(triVec.verts[n].nrm|triVec.verts[n].nrm);
}

void MeshAnim::Load(const string &fileName) {
	std::filebuf fb;
	if(!fb.open (fileName.c_str(),std::ios::in)) return;

	std::istream in(&fb);

	numJoints=~0; numFrames=~0;
	int countH=0,countB=0,numAnimComp=~0;
	string token;

	while(!in.eof()) {
		in >> token;
		if(token=="hierarchy") {
			in >> token; assert(token=="{");

			while(true) {
				if(in.eof()) ThrowException("Error while loading mesh anim from ",fileName);
				in >> token;
				if(token=="}") break;

				Hierarchy &h=hierarchy[countH++];
				h.name=token; in >> h.parent >> h.numComp >> h.frameIdx;
				h.name = h.name.substr(1,h.name.length()-2);
			}
		}
		else if(token=="bounds") {
			in >> token; assert(token=="{");
			while(true) {
				if(in.eof()) ThrowException("Error while loading mesh anim from ",fileName);
				in >> token;
				if(token=="}") break;

				BBox &box=boxes[countB++];
				string p1,p2;
				p1=token; in >> box.min.x >> box.min.z >> box.min.y >> p2; assert(p1=="("&&p2==")");
				in >> p1 >> box.max.x >> box.max.z >> box.max.y >> p2; assert(p1=="("&&p2==")");
			}
		}
		else if(token=="baseframe") {
			in >> token; assert(token=="{");
			framePos.resize((numFrames+1)*numJoints);
			frameRot.resize((numFrames+1)*numJoints);
			Vec3f *pos=&framePos[numFrames*numJoints];
			Vec4f *rot=&frameRot[numFrames*numJoints];
			while(true) {
				if(in.eof()) ThrowException("Error while loading mesh anim from ",fileName);
				in >> token;
				if(token=="}") break;

				string p1,p2;
				p1=token; in >> pos->x >> pos->z >> pos->y >> p2; assert(p1=="("&&p2==")");
				in >> p1 >> rot->x >> rot->z >> rot->y >> p2; assert(p1=="("&&p2==")");
				rot->w = 1.0f-rot->x*rot->x-rot->y*rot->y-rot->z*rot->z;
				rot->w=rot->w<0.0f?0.0f:sqrt(rot->w);
				pos++; rot++;
			}
		}
		else if(token=="frame") {
			int idx; in >> idx;
			in >> token; assert(token=="{");
			Vec3f *pos=&framePos[idx*numJoints],*basePos=&framePos[numFrames*numJoints];
			Vec4f *rot=&frameRot[idx*numJoints],*baseRot=&frameRot[numFrames*numJoints];
			Hierarchy *h=&hierarchy[0];

			int n=0;
			while(true) {
				if(in.eof()) ThrowException("Error while loading mesh anim from ",fileName);

				if(h[n].numComp&1) in >> pos[n].x; else pos[n].x=basePos[n].x;
				if(h[n].numComp&2) in >> pos[n].z; else pos[n].z=basePos[n].z;
				if(h[n].numComp&4) in >> pos[n].y; else pos[n].y=basePos[n].y;
				if(h[n].numComp&8) in >> rot[n].x; else rot[n].x=baseRot[n].x;
				if(h[n].numComp&16) in >> rot[n].z; else rot[n].z=baseRot[n].z;
				if(h[n].numComp&32) in >> rot[n].y; else rot[n].y=baseRot[n].y;
				rot[n].w=1.0f-rot[n].x*rot[n].x-rot[n].y*rot[n].y-rot[n].z*rot[n].z;
				rot[n].w=rot[n].w<0.0f?0.0f:sqrt(rot[n].w);

				if(h[n].parent!=-1) {
					pos[n]=Trans(rot[h[n].parent],pos[n])+pos[h[n].parent];
					rot[n]=MulQuat(rot[h[n].parent],rot[n]);
				}

				if(++n==numJoints) break;
			}
		}
		else if(token=="numFrames") { in >> numFrames; boxes.resize(numFrames+1); }
		else if(token=="numJoints") { in >> numJoints; hierarchy.resize(numJoints+1); }
		else if(token=="frameRate") in >> frameRate;
		else if(token=="numAnimatedComponents") in >> numAnimComp;
	}

	{
		Vec3f *pos=&framePos[numFrames*numJoints];
		Vec4f *rot=&frameRot[numFrames*numJoints];
		Hierarchy *h=&hierarchy[0];
		for(int n=0;n<numJoints;n++) if(h[n].parent!=-1) {
			pos[n]=Trans(rot[h[n].parent],pos[n])+pos[h[n].parent];
			rot[n]=MulQuat(rot[h[n].parent],rot[n]);
		}
	}
}

void MeshAnim::GetFrame(float pos,Vec3f *oPos,Vec4f *oQuat) const {
	int nFrame=pos*(frameRate);
	int frm1=nFrame%numFrames,frm2=(nFrame+1)%numFrames;
	pos=pos*frameRate; pos-=int(pos);

	const Vec3f *sPos[2]={&framePos[frm1*numJoints],&framePos[frm2*numJoints]};
	const Vec4f *sRot[2]={&frameRot[frm1*numJoints],&frameRot[frm2*numJoints]};

	for(int n=0;n<numJoints;n++) {
		oPos[n]=Lerp(sPos[0][n],sPos[1][n],pos);
		oQuat[n]=Slerp(sRot[0][n],sRot[1][n],pos);
	}
	for(int n=1;n<numJoints;n++) oPos[n]-=oPos[0];
	oPos[0]=Vec3f(0.0f,0.0f,0.0f);
}

void MeshAnim::GetBaseFrame(Vec3f *pos,Vec4f *quat) const {
	const Vec3f *sPos=&framePos[numFrames*numJoints];
	const Vec4f *sRot=&frameRot[numFrames*numJoints];

	for(int n=0;n<numJoints;n++) { pos[n]=sPos[n]; quat[n]=sRot[n]; }
	for(int n=1;n<numJoints;n++) pos[n]-=pos[0];
	pos[0]=Vec3f(0.0f,0.0f,0.0f);
}

