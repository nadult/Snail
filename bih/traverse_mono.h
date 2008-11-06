
namespace bih {

	template <class Element>
	typename Tree<Element>::template ReturnType<float,1>::Result Tree<Element>::TraverseMono
		(const Vec3p &rOrigin,const Vec3p &tDir) const
	{
		TreeStats stats;
		stats.TracingRay();
		typename ReturnType<float,1>::Result out;
		out.Distance(0)=1.0f/0.0f;

		Vec3p rDir=Vec3p(tDir.x+0.000000000001f,tDir.y+0.000000000001f,tDir.z+0.000000000001f);
		Vec3p invDir=VInv(rDir);

		int dirMask=SignMask(floatq(invDir.m));
		float tMin=0.0f,tMax=out.Distance(0);

		struct Locals { float tMin,tMax; u32 idx; } stackBegin[maxLevel+2],*stack=stackBegin;
		const Node *node,*node0=&nodes[0];
		int idx=0;

		{
			Vec3p ttMin=(pMin-rOrigin)*invDir;
			Vec3p ttMax=(pMax-rOrigin)*invDir;
			if(dirMask&1) Swap(ttMin.x,ttMax.x);
			if(dirMask&2) Swap(ttMin.y,ttMax.y);
			if(dirMask&4) Swap(ttMin.z,ttMax.z);

			tMax=Min(Min(ttMax.x,ttMax.y),tMax);
			tMax=Min(ttMax.z,tMax);
			
			tMin=Max(Max(ttMin.x,ttMin.y),tMin);
			tMin=Max(ttMin.z,tMin);
		}

		while(true) {
			stats.LoopIteration();
			if(tMin>tMax) goto POP_STACK;
		
			if(idx&Node::leafMask) {
				idx&=Node::idxMask;
				{
					stats.Intersection();
					typename ReturnType<float,1>::BaseIntersection tOut=elements[idx].Collide(rOrigin,tDir);
					if(tOut.Distance(0)<out.Distance(0)) {
						out.Distance(0)=tOut.Distance(0);
						out.Object(0)=idx;
						if(ReturnType<float,1>::Result::flags&ifElement) out.Element(0)=tOut.Element(0);
					}
				}
POP_STACK:
				if(stack==stackBegin) break;
				stack--;
				tMin=stack->tMin;
				tMax=Min(stack->tMax,out.Distance(0));
				idx=stack->idx;
				continue;
			}

			idx&=Node::idxMask;
			node=node0+idx;
			int axis=node->Axis();
			int nidx=dirMask&(1<<axis)?1:0,fidx=nidx^1;

			float near,far; {
				float start=(&rOrigin.x)[axis],inv=(&invDir.x)[axis];
				near=(node->clip[nidx]-start)*inv;
				far =(node->clip[fidx]-start)*inv;
			}

			if(tMin>near) {
				if(tMax<far) goto POP_STACK;

				tMin=Max(tMin,far);
				idx=node->val[fidx];
				continue;
			}
			if(tMax<far) {
				tMax=Min(tMax,near);
				idx=node->val[nidx];
				continue;
			}

			stack->tMin=Max(tMin,far);
			stack->tMax=tMax;
			stack->idx=node->val[fidx];
			stack++;

			tMax=Min(tMax,near);
			idx=node->val[nidx];
		}

		//if(sizeof(Element)==64&&output.stats) output.stats->Update(stats);
		return out;
	}

}


