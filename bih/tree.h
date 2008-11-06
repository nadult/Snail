#ifndef RTRACER_BIH_TREE_H
#define RTRACER_BIH_TREE_H

#include "ray_group.h"
#include "tree_stats.h"
#include "context.h"
#include "triangle.h"

namespace bih {

	// Liscie nie sa przechowywane w drzewie
	// Zamiast odnosnika do liscia jest od razu odnosnik do obiektu
	class Node {
	public:
		enum { leafMask=1<<29, idxMask=0x1fffffff };

		inline i32 Child1() const { return val[0]&0x1fffffff; }
		inline i32 Child2() const { return val[1]&0x1fffffff; }
		inline i32 Child(uint idx) const { return val[idx]&0x1fffffff; }

		inline bool Child1IsLeaf() const { return val[0]&leafMask; }
		inline bool Child2IsLeaf() const { return val[1]&leafMask; }
		inline bool ChildIsLeaf(uint idx) const { return val[idx]&leafMask; }

		inline int Axis() const { return (val[0]>>30); }
		inline float ClipLeft() const { return clip[0]; }
		inline float ClipRight() const { return clip[1]; }

		inline bool DenseNode() const { return val[1]&(1<<30); }

		float clip[2];
		u32 val[2];
	};

	class Index {
	public:
		Index() { }
		Index(int i,const Vec3f &mi,const Vec3f &ma,float mul) :min(mi),max(ma),idx(i) {
			Vec3f vsize=max-min;
			size=Max(vsize.x,Max(vsize.y,vsize.z))*mul;
		}
		bool operator<(const Index &i) const { return idx<i.idx; }

		Vec3f min,max;
		float size;
		i32 idx;
	};

//	void GenBIHIndices(const TriVector &tris,vector<Index> &out,float maxSize,uint maxSplits);

	void FindSplit(const vector<Index> &indices,const Vec3f &min,const Vec3f &max,int &outAxis,float &outSplit);
	bool SAH(const vector<Index> &indices,const Vec3f &min,const Vec3f &max,int &outAxis,float &outSplit);
	
	template <class Element>
	void SplitIndices(const vector<Element,AlignedAllocator<Element> > &elements,
						vector<Index> &inds,int axis,float pos,float maxSize) {
	//	if(inds.size()<=(maxSize>0.0f?16:64)) return;
		if(inds.size()<=2) return;
		maxSize*=1.5f;

		for(int n=0,end=inds.size();n<end;n++) {
			Index &idx=inds[n];
			if(idx.size<maxSize) continue;
			if((&idx.min.x)[axis]>=pos||(&idx.max.x)[axis]<=pos) continue;

			const Element &elem=elements[idx.idx];

			Vec3f min1=idx.min,min2=idx.min,max1=idx.max,max2=idx.max;
			(&max1.x)[axis]=(&min2.x)[axis]=pos;

			idx=Index(idx.idx,min1,max1,1.0f);
			inds.push_back(Index(idx.idx,min2,max2,1.0f));
		}
	}

	void SplitIndices(const TriVector &tris,vector<Index> &inds,int axis,float pos,float maxSize);

//	void OptimizeIndices(vector<Index> &indices);

	template <class Element_>
	class Tree: public RefCounter {
	public:
		typedef Element_ Element;
		typedef vector<Element,AlignedAllocator<Element> > ElementContainer;
		//1 for tree of triangles, 2 for tree of trees of triangles etc
		enum { complexity=Element::complexity+1 };
		enum { maxLevel=60 };

		template <class real,int packetSize>
		struct ReturnType {
			typedef typename Element_::template ReturnType<real,packetSize>::Result BaseIntersection;
			typedef Intersection<real,packetSize,BaseIntersection::flags|ifObject> Result; };
		
		Tree() { }

		BBox GetBBox() const {
			return BBox(pMin,pMax);
		}

		Vec3f FlatNormals(u32 elementId,u32 subElementId) const {
			return elements[elementId].Nrm(subElementId);
		}

		Tree(const ElementContainer &elements);

		void PrintInfo() const;
		uint FindSimilarParent(vector<u32> &parents,uint nNode,uint axis) const;
		void Build(vector<Index> &indices,vector<u32> &parents,uint nNode,const Vec3f &min,const Vec3f &max,uint level,bool);


	protected:
		template <int packetSize,bool sharedOrigin,OutputType outputType>
		class TravContext {
			const RayGroup<packetSize,sharedOrigin,1> &rays;
			Output<outputType,f32x4,i32x4> output;
			
			int instanceId;
			
		public:
			TravContext(const RayGroup<packetSize,sharedOrigin,1> &r,Output<outputType,f32x4,i32x4> out) :rays(r),output(out) {
			}
		};

	public:	
/*		template <bool sharedOrigin,template <int> class Selector>
		typename ReturnType<float,1>::Result TraversePacket(
						bool filterSigns,const RayGroup<1,sharedOrigin,1> &rays,
						const Selector<1> &selector) const {
			int bitMask=selector.BitMask(0);

			if(bitMask==0x0f) {
				const Vec3q &dir=rays.Dir(0);
				int msk=_mm_movemask_ps(_mm_shuffle_ps(_mm_shuffle_ps(dir.x.m,dir.y.m,0),dir.z.m,0+(2<<2)))&7;
				int signMask=filterSigns?GetVecSign(rays.Dir(0)):msk;
				
				if(signMask!=8) {
					TraversePacket<1,sharedOrigin>(rays,instanceId,msk);
					return;
				}
			}

			::Output<otNormal,float,u32> out((float*)output.dist,(u32*)output.object,(u32*)output.element,output.stats);
			const Vec3q &orig=rays.Origin(0),&dir=rays.Dir(0);

//			printf("%d\n",bitMask);			
			for(int k=0;k<4;k++) {
				if(!(bitMask&(1<<k))) continue;

				int o=sharedOrigin?0:k;
				TraverseMono(Vec3p(orig.x[o],orig.y[o],orig.z[o]),Vec3p(dir.x[k],dir.y[k],dir.z[k]),out,instanceId);
				out.dist++; out.object++; out.element++;
			}
		} */

		template <bool sharedOrigin>
		typename ReturnType<f32x4,1>::Result TraversePacket(bool filterSigns,const RayGroup<1,sharedOrigin,1> &rays) const {
			const Vec3q &dir=rays.Dir(0);
			int msk=_mm_movemask_ps(_mm_shuffle_ps(_mm_shuffle_ps(dir.x.m,dir.y.m,0),dir.z.m,0+(2<<2)))&7;
			int signMask=filterSigns?GetVecSign(rays.Dir(0)):msk&7;

			typedef typename ReturnType<f32x4,1>::Result Result;
			Result out;
				
			if(signMask!=8) out=TraversePacket<1,sharedOrigin>(rays,msk);
			else {
				const Vec3q &orig=rays.Origin(0);
				for(int q=0;q<4;q++) {	
					int o=sharedOrigin?0:q;
					typename ReturnType<float,1>::Result tOut=	
						TraverseMono(Vec3p(orig.x[o],orig.y[o],orig.z[o]),Vec3p(dir.x[q],dir.y[q],dir.z[q]));
					if(Result::flags&ifDistance) out.Distance(0)[q]=tOut.Distance(0);
					if(Result::flags&ifObject  ) out.Object  (0)[q]=tOut.Object  (0);
					if(Result::flags&ifElement ) out.Element (0)[q]=tOut.Element (0);
				}
			}

			return out;
		}

/*		template <bool sharedOrigin,OutputType outputType,template <int> class Selector,int packetSize>
		void TraversePacket(bool filterSigns,const RayGroup<packetSize,sharedOrigin,1> &rays,
						const Selector<packetSize> &selector,Output<outputType,f32x4,i32x4> output,int instanceId) const {
			
			bool selectorsFiltered=(packetSize<=16);
//			selectorsFiltered=true;

			for(int n=0;n<packetSize/4;n++) if(selector.BitMask4(n)!=0x0f0f0f0f) {
				selectorsFiltered=0;
				break;
			}
				
			if(selectorsFiltered) {
				bool signsFiltered=1;

				const Vec3q &dir=rays.Dir(0);
				int msk=_mm_movemask_ps(_mm_shuffle_ps(_mm_shuffle_ps(dir.x.m,dir.y.m,0),dir.z.m,0+(2<<2)));
				int signMask=msk&7;

				if(filterSigns) {					
					for(int n=0;n<packetSize;n++) if(GetVecSign(rays.Dir(n))!=signMask) {
						signsFiltered=0;
						break;
					}
				}
			
				if(signsFiltered) {
					TraversePacket<packetSize,sharedOrigin>(rays,output,instanceId,signMask);
					return;
				}
			}	
			
			Selector<packetSize/4> qSelectors[4];
			selector.SplitTo4(qSelectors);
			Output<outputType,f32x4,i32x4> tOutput(output.dist,output.object,output.element,output.stats);
				
			for(int q=0;q<4;q++) {
				TraversePacket(filterSigns,RayGroup<packetSize/4,sharedOrigin,1>(rays,q*(packetSize/4)),
								qSelectors[q],tOutput,instanceId);

				tOutput.dist+=(packetSize/4);
				tOutput.object+=(packetSize/4);
				tOutput.element+=(packetSize/4);
			}
		} */
	
		template <bool sharedOrigin,int packetSize>
		typename ReturnType<f32x4,packetSize>::Result TraversePacket(bool filterSigns,const RayGroup<packetSize,sharedOrigin,1> &rays) const {
		
			typename ReturnType<f32x4,packetSize>::Result out;	
			bool signsFiltered=1;

			const Vec3q &dir=rays.Dir(0);
			int msk=_mm_movemask_ps(_mm_shuffle_ps(_mm_shuffle_ps(dir.x.m,dir.y.m,0),dir.z.m,0+(2<<2)));
			int signMask=msk&7;

			if(filterSigns) {					
				for(int n=0;n<packetSize;n++) if(GetVecSign(rays.Dir(n))!=signMask) {
					signsFiltered=0;
					break;
				}
			}
			
			if(signsFiltered) out=TraversePacket<packetSize,sharedOrigin>(rays,signMask);
			else {
				for(int q=0;q<4;q++) {
					typename ReturnType<f32x4,packetSize/4>::Result tOut=
						TraversePacket(filterSigns,RayGroup<packetSize/4,sharedOrigin,1>(rays,q*(packetSize/4)));
					out.Insert(tOut,q);
				}
			}

			return out;
		}
		
		template <int packetSize,bool sharedOrigin>
		typename ReturnType<f32x4,packetSize>::Result TraversePacket(const RayGroup<packetSize,sharedOrigin,1> &rays,int dirMask) const;
		
		typename ReturnType<float,1>::Result TraverseMono(const Vec3p &rOrigin,const Vec3p &tDir) const;

		float avgSize;
		Vec3f pMin,pMax;
		vector<Node> nodes;
		ElementContainer elements;

		int objectId;
		float maxDensity;
		bool split;
	};

	template <class BaseTree_>
	class BIHBox {
	public:
		typedef BaseTree_ BaseTree;
		enum { complexity=BaseTree::complexity };
	

		template <class real,int packetSize>
		struct ReturnType {
			typedef typename BaseTree_::template ReturnType<real,packetSize>::Result BaseIntersection;
			typedef Intersection<real,packetSize,BaseIntersection::flags|ifElement> Result; };

		BIHBox() :tree(0) { }
		BIHBox(const BaseTree *tr,Matrix<Vec4f> m,const BBox &b) :trans(m),invTrans(Inverse(m)),bBox(b),tree(tr) { }

		Vec3f Nrm(int subElementId) const {
			return trans&tree->FlatNormals(subElementId,0);
		}

		Vec3f BoundMin() const { return bBox.min; }
		Vec3f BoundMax() const { return bBox.max; }
		BBox GetBBox() const { return bBox; }

		typename ReturnType<float,1>::Result Collide(const Vec3f &rOrig,const Vec3f &rDir) const {
			assert(tree);
			typename ReturnType<float,1>::Result out;

			typename ReturnType<float,1>::BaseIntersection tOut=tree->TraverseMono(invTrans*rOrig,invTrans&rDir);

			out.Distance(0)=tOut.Distance(0);
			out.Element(0)=tOut.Object(0);
		//	out.object[0]=0;

			return out;
		}

		template <int packetSize,bool sharedOrigin,bool precompInv>
		typename ReturnType<f32x4,packetSize>::Result Collide(const RayGroup<packetSize,sharedOrigin,precompInv> &rays) const {
			assert(tree);

			// TODO: nie dziala dla precompInv==0 

			typename ReturnType<f32x4,packetSize>::Result out;

			bool test=0; test=1;
//			if(sharedOrigin) test=bBox.TestIP<packetSize>(rays.Origin(0),rays.idir,output.dist);
//			else for(int q=0;q<packetSize&&!test;q++)
//				test|=bBox.TestI(rays.Origin(sharedOrigin?0:q),rays.idir[q],output.dist[q]);	
			if(test) {
				Vec3q origin[packetSize],dir[packetSize],idir[packetSize];
				for(int q=0;q<(sharedOrigin?1:packetSize);q++)
					origin[q]=invTrans*rays.Origin(q);
				for(int q=0;q<packetSize;q++) {
					dir[q]=invTrans&rays.Dir(q);
					idir[q]=VInv(dir[q]);
				}

				typename ReturnType<f32x4,packetSize>::BaseIntersection tOut=
					tree->TraversePacket(true,RayGroup<packetSize,sharedOrigin,1>(origin,dir,idir));
				for(int q=0;q<packetSize;q++) out.Distance(q)=tOut.Distance(q);
				for(int q=0;q<packetSize;q++) out.Element(q)=tOut.Object(q);
			//	for(int q=0;q<packetSize;q++) out.object[q]=0;
			}
			else {
				for(int q=0;q<packetSize;q++)
					out.Distance(q)=1.0f/0.0f;
			}

			return out;
		}

//	private:
		Matrix<Vec4f> trans,invTrans;
		BBox bBox;
		const BaseTree_ *tree;
	};

}

#include "bih/tree_impl.h"
#include "bih/traverse_mono.h"
#include "bih/traverse_packet.h"


#endif


