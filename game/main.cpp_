#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include "rtbase.h"
#include "triangle.h"

#include <iostream>
#include "camera.h"

#include "gl_window.h"
#include "formats/loader.h"
#include "base_scene.h"
#include "font.h"

#include "render.h"
#include "shading/material.h"

#include "bih/tree.h"
#include "tree_box.h"
#include "scene.h"
#include "mesh.h"
#include "scene_builder.h"
#include "frame_counter.h"

template <class T>
class FreePtr
{
public:
	FreePtr(T *p) :ptr(p) { }
	FreePtr() :ptr(0) { }
	~FreePtr() { if(ptr) delete ptr; }
	void operator=(T *p) { if(ptr) delete ptr; ptr=p; }
	operator T*() { return ptr; }
	operator const T*() const { return ptr; }
	T* operator ->() { return ptr; }
	const T* operator->() const { return ptr; }


protected:
	T *ptr;

private:
	FreePtr(const FreePtr&);
};

class Physics {
public:
	Physics() {
		btVector3 aabbMin(-2000,-2000,-2000),aabbMax(2000,2000,2000);

		colConfig=new btDefaultCollisionConfiguration();
		dispatcher=new btCollisionDispatcher(colConfig);
		broadphase=new btDbvtBroadphase();
		solver=new btSequentialImpulseConstraintSolver;

		world=new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,colConfig);
		world->setGravity(btVector3(0,-550,0));
		btAxisSweep3* sweepBP = new btAxisSweep3(aabbMin,aabbMax);
	

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-50,0));
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(2000.),btScalar(10.),btScalar(2000.)));

		{

			btConvexShape* capsule=new btCapsuleShape(30.0f,60.0f);
			btTransform transform; transform.setIdentity();
			transform.setOrigin (btVector3(-73.0f,111.0f,-504.0f));
			btScalar mass(10.f);

			btVector3 localInertia(0,0,0);
//			capsule->calculateLocalInertia(mass,localInertia);
//			localInertia[0]=localInertia[2]=0.0f;

			//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
			btDefaultMotionState* myMotionState=new btDefaultMotionState(transform);
			btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,capsule,localInertia);
			rbInfo.m_friction=1.0f;
			rbInfo.m_linearSleepingThreshold=0.0f;
			rbInfo.m_angularSleepingThreshold=0.0f;

			player=new btRigidBody(rbInfo);
		//	body->setActivationState(ISLAND_SLEEPING);

			world->addRigidBody(player);
		//	body->setActivationState(ISLAND_SLEEPING);
			pFront=Vec3f(0,0,1);
		}
	}

	Vec3f pFront;

	void GetCam(Vec3f &pos,Vec3f &up,Vec3f &front) {
		btTransform characterWorldTrans;

		//look at the vehicle
		characterWorldTrans = player->getWorldTransform();
	//	btVector3 up_ = characterWorldTrans.getBasis()[1];
	//	btVector3 backward = -characterWorldTrans.getBasis()[2];
	//	up_.normalize (); backward.normalize ();

		btVector3 position= characterWorldTrans.getOrigin();
	/*	up=Vec3f(up_[0],up_[1],up_[2]);
		front=Vec3f(-backward[0],-backward[1],-backward[2]); */
		
		up=Vec3f(0,1,0);
		front=pFront;

		pos=Vec3f(position[0],position[1],position[2]);
		pos+=up*35.0f;
	}

	void Rotate(float x) { pFront=RotateY(x)*pFront; }

//	void Rotate(float x) { player->applyTorque(btVector3(0,x,0)); }
	void Move(Vec3f v) { player->applyCentralImpulse(btVector3(v.x,v.y,v.z)); }

	void AddTris(const TriVector &tris) {
		btVector3 aabbMin(-10000,-10000,-10000),aabbMax(10000,10000,10000);

		btTriangleIndexVertexArray *meshData; {
			int *indices=new int[tris.size()*3];
			Vec3f *verts=new Vec3f[tris.size()*3];
			for(int n=0;n<tris.size();n++) {
				indices[n*3+0]=n*3+0;
				indices[n*3+1]=n*3+1;
				indices[n*3+2]=n*3+2;
				verts[n*3+0]=tris[n].P1();
				verts[n*3+1]=tris[n].P2();
				verts[n*3+2]=tris[n].P3();
			}
			meshData=new btTriangleIndexVertexArray(tris.size(),indices,12,tris.size()*3,
													(btScalar*)verts,12);
		}

		btBvhTriangleMeshShape *groundShape=new btBvhTriangleMeshShape(meshData,1,aabbMin,aabbMax);
		
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,0,0));

		//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
		{
			btScalar mass(0.);

			//rigidbody is dynamic if and only if mass is non zero, otherwise static
			bool isDynamic = (mass != 0.f);

			btVector3 localInertia(0,0,0);
			if (isDynamic)
				groundShape->calculateLocalInertia(mass,localInertia);

			//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
			btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
			btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
			btRigidBody* body=new btRigidBody(rbInfo);

			//add the body to the dynamics world
			world->addRigidBody(body);
		}
	}

	void Tick(float time) { world->stepSimulation(time,10); }

//private:
	btAlignedObjectArray<btCollisionShape*>	collisionShapes;
	FreePtr<btBroadphaseInterface> broadphase;
	FreePtr<btCollisionDispatcher> dispatcher;
	FreePtr<btConstraintSolver>	solver;
	FreePtr<btDefaultCollisionConfiguration> colConfig;
	FreePtr<btDynamicsWorld> world;
	FreePtr<btRigidBody> player;
};

typedef bih::Tree<TriangleVector> StaticTree;
typedef bih::Tree<TreeBoxVector<StaticTree> > FullTree;

namespace {

	template <class Scene>
	void SetMaterials(Scene &scene,const BaseScene &base,string texPath) {
		scene.materials.clear();
		scene.materials.resize(base.matNames.size()+1);
		for(std::map<string,int>::const_iterator it=base.matNames.begin();it!=base.matNames.end();++it) {
			string name=it->first==""?"":texPath+it->first;

			try {
				scene.materials[it->second]=typename Scene::PMaterial(shading::NewMaterial(name));
			}
			catch(const Exception &ex) {
				std::cout << ex.what() << '\n';
				scene.materials[it->second]=scene.materials.front();
			}
		}
		scene.materials.back()=typename Scene::PMaterial(shading::NewMaterial("scenes/doom3/imp/texture.tga"));

		if(scene.materials.size()>126) {
			scene.materials[126]->flags|=shading::Material::fReflection;
			scene.materials[ 70]->flags|=shading::Material::fRefraction;
		}
	}

	vector<Light> GenLights() {
		vector<Light> out;

		float pos=float(gVals[5])*0.01f;

		out.push_back( Light(Vec3f(-78.0f,110.0f,-531.0f),Vec3f(1,1,0.7),1000.0f) );
	//	out.push_back(Light(Vec3f(-600.0f,144.0f,-341.0f),Vec3f(0.3,0.6,1.0),800.0f));
	//	out.push_back(Light(Vec3f(407.0f,209.64f,1634.0f),Vec3f(1,1,1),1000.0f));

		return out;
	}

}

namespace game {

	int main(int argc, char **argv) {
		CameraConfigs camConfigs;
		try { Loader("scenes/cameras.dat") & camConfigs; } catch(...) { }

		Mesh mesh;
		MeshAnim meshAnim;
		mesh.Load("scenes/doom3/imp/imp.md5mesh");
		meshAnim.Load("scenes/doom3/imp/walk1.md5anim");

		StaticTree meshTree;

		int resx=800,resy=600;
#ifndef NDEBUG
		resx/=2; resy/=2;
#endif
		bool fullscreen=0;
		int threads=4;
		const char *modelFile="doom3/admin.proc";
		Options options;
		bool flipNormals=1;
		string texPath="/mnt/Data/data/doom3/";

		for(int n=1;n<argc;n++) {
				 if(string("-res")==argv[n]&&n<argc-2) { resx=atoi(argv[n+1]); resy=atoi(argv[n+2]); n+=2; }
			else if(string("-threads")==argv[n]&&n<argc-1) { threads=atoi(argv[n+1]); n+=1; }
			else if(string("-fullscreen")==argv[n]) { fullscreen=1; }
			else if(string("+flipNormals")==argv[n]) flipNormals=1;
			else if(string("-flipNormals")==argv[n]) flipNormals=0;
			else if(string("-texPath")==argv[n]) { texPath=argv[n+1]; n++; }
			else {
				if(argv[n][0]=='-') printf("Unknown option: ",argv[n]);
				else modelFile=argv[n];
			}
		}

		printf("Threads/cores: %d/%d\n\n",threads,4);

		printf("Loading...\n");
		double buildTime=GetTime();
		BaseScene baseScene; {
			string fileName=string("scenes/")+modelFile;
			if(fileName.find(".proc")!=string::npos)
				baseScene.LoadDoom3Proc(fileName);
			else if(fileName.find(".obj")!=string::npos)
				baseScene.LoadWavefrontObj(fileName);

			if(flipNormals) baseScene.FlipNormals();
			for(int n=0;n<baseScene.objects.size();n++)
				baseScene.objects[n].Repair();
		//	baseScene.GenNormals();
		//	baseScene.Optimize();
		}

		SceneBuilder<StaticTree> builder; {
			vector<BaseScene::Object> objects;

			for(int n=0;n<baseScene.objects.size();n++) {
				const BaseScene::Object &obj=baseScene.objects[n];
				bool contained=0;
				for(int k=0;k<objects.size();k++) {
					BaseScene::Object &tObj=objects[k];
					if(1||obj.GetBBox().Contains(tObj.GetBBox(),1.5f)||tObj.GetBBox().Contains(obj.GetBBox(),1.5f)) {
						contained=1;
						tObj.Join(obj);
						break;
					}
				}
				if(!contained) objects.push_back(baseScene.objects[n]);

			}

			int count=0;	
			for(int o=0;o<objects.size();o++) {
				BaseScene::Object &obj=objects[o];
				obj.Optimize();
				TriangleVector vec=obj.ToTriangleVector();
				if(vec.size()) {
					builder.AddObject(new StaticTree(vec),obj.GetTrans(),obj.GetBBox());
					builder.AddInstance(count++,Identity<>());
				}
			}
		}

		Image img(resx,resy,16);

		for(int n=0;n<10;n++) gVals[n]=1;
		gVals[2]=0; gVals[4]=0; gVals[3]=0;

		Scene<FullTree> scene;
		SetMaterials(scene,baseScene,texPath);
		mesh.SetMaterial(scene.materials.size()-1);

		vector<Light> lights=GenLights();
				
		Camera cam;
		if(!camConfigs.GetConfig(string(modelFile),cam))
			cam.pos=scene.geometry.GetBBox().Center();

		buildTime=GetTime()-buildTime;
		std::cout << "Build time: " << buildTime << '\n';

		GLWindow out(resx,resy,fullscreen);
		Font font;

		bool lightsEnabled=1;
		float speed; {
			scene.geometry.Construct(builder.ExtractElements());
			Vec3p size=scene.geometry.GetBBox().Size();
			speed=(size.x+size.y+size.z)*0.0025f;
		}

		FrameCounter frmCounter;
		Physics physics;
		physics.AddTris(baseScene.ToTriVector());

		while(out.PollEvents()) {
			frmCounter.NextFrame();

			if(out.KeyUp(Key_esc)) break;
			if(out.KeyDown('K')) img.SaveToFile("out/output.tga");
			if(out.KeyDown('O')) options.reflections^=1;
			if(out.KeyDown('I')) options.rdtscShader^=1;
			if(out.KeyDown('P')) {
				camConfigs.AddConfig(string(modelFile),cam);
				Saver("scenes/cameras.dat") & camConfigs;
				cam.Print();
			}
			if(out.KeyDown('L')) {
				printf("Lights %s\n",lightsEnabled?"disabled":"enabled");
				lightsEnabled^=1;
			}
			if(out.KeyDown('J')) {
				Vec3f colors[4]={Vec3f(1,1,1),Vec3f(0.2,0.5,1),Vec3f(0.5,1,0.2),Vec3f(0.7,1.0,0.0)};

				lights.push_back(Light(cam.pos,colors[rand()&3],800.0f));
			}

			{
				btVector3 pv=physics.player->getLinearVelocity();
				float pSpeed=Length(Vec3f(pv[0],pv[1],pv[2]))/physics.player->getInvMass();

				Vec3f pos,up,front,right;
				physics.GetCam(pos,up,front);
				right=up^front;
				right+=up*0.1f;
				front+=up*0.1f;

				float maxSpeed=Min(out.Key(Key_lshift)?3000:2000.0f,pSpeed+speed*50.0f);
				float tspeed=maxSpeed-pSpeed;

				bool moving=0;
				if(out.Key('W')) { moving=1; physics.Move( front*tspeed); }
				if(out.Key('S')) { moving=1; physics.Move(-front*tspeed); }
				if(out.Key('A')) { moving=1; physics.Move(-right*tspeed); }
				if(out.Key('D')) { moving=1; physics.Move( right*tspeed); }
				if(!moving) {
					physics.Move(Vec3f(-pv[0],-pv[1]*0.1f,-pv[2])*0.25f);
				}

				if(out.MouseKey(0)) {
					Vec3f rot=out.MouseMove();
					physics.Rotate(-rot.x*0.01f);
				}
			}

		//	if(out.KeyDown('Y')) { printf("splitting %s\n",scene.tree.split?"off":"on"); scene.tree.split^=1; }

			if(out.Key(Key_lctrl)) {
		//		if(out.KeyDown('1')&&scene.lights.size()>=1) scene.lights[0].pos=cam.pos;
		//		if(out.KeyDown('2')&&scene.lights.size()>=2) scene.lights[1].pos=cam.pos;
		//		if(out.KeyDown('3')&&scene.lights.size()>=3) scene.lights[2].pos=cam.pos;
			}

			if(out.KeyDown(Key_f2)) { gVals[1]^=1; printf("Val 2 %s\n",gVals[1]?"on":"off"); }
			if(out.KeyDown(Key_f3)) { gVals[2]^=1; printf("Val 3 %s\n",gVals[2]?"on":"off"); }
			if(out.KeyDown(Key_f4)) { gVals[3]^=1; printf("Val 4 %s\n",gVals[3]?"on":"off"); }
			if(out.KeyDown(Key_f5)) { gVals[4]^=1; printf("Toggled shading\n"); }
			if(out.KeyDown(Key_f6)) { gVals[5]^=1; printf("Val 5 %s\n",gVals[5]?"on":"off"); }

			double buildTime=GetTime(); {
				static float pos=0.0f; if(out.Key(Key_space)) pos+=0.025f;
				SceneBuilder<StaticTree> temp=builder;
			//	for(int n=0;n<temp.instances.size();n++) {
			//		SceneBuilder<StaticTree>::Instance &inst=temp.instances[n];
			//		inst.trans.w=inst.trans.w+Vec4f(0.0f,sin(pos+n*n)*5.0f*speed,0.0f,0.0f);
			//	}
				mesh.Animate(meshAnim,pos);
				meshTree.Construct(mesh.triVec,1);

				temp.AddObject(&meshTree,Identity<>(),meshTree.GetBBox());
				for(int x=-1;x<2;x++) for(int z=-1;z<2;z++)
					temp.AddInstance(temp.objects.size()-1,
							RotateY(3.1415f)*Translate(Vec3f(132+x*50,0,z*50-346)));
				scene.geometry.Construct(temp.ExtractElements(),1);
				BBox box=meshTree.GetBBox();

				vector<Light> tLights=lightsEnabled?lights:vector<Light>();
				for(int n=0;n<tLights.size();n++) {
					tLights[n].pos += Vec3f(sin(pos+n*n),cos(pos+n*n),sin(pos-n*n)*cos(pos+n*n))*speed;
				}
				scene.lights=tLights;
			buildTime=GetTime()-buildTime; }

			scene.Update();
			
			double time=GetTime();
			TreeStats<1> stats;
			{
				physics.GetCam(cam.pos,cam.up,cam.front);
				cam.right=cam.up^cam.front;
			}

			stats=Render(scene,cam,img,options,threads);

			out.RenderImage(img);

			time=GetTime()-time;

			font.BeginDrawing(resx,resy);
			font.SetSize(Vec2f(30,20));
				font.PrintAt(Vec2f(0,0),stats.GenInfo(resx,resy,time*1000.0,buildTime*1000.0));
				font.PrintAt(Vec2f(0,20),"FPS (game mode):",int(frmCounter.FPS()));
				font.PrintAt(Vec2f(0,40),"Lights: ",lightsEnabled?int(lights.size()):0);

			font.FinishDrawing();
			physics.Tick(time);

			out.SwapBuffers();
		}

		return 0;
	}

}
