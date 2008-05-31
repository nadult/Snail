#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int count;

void Line(const char *str) {
	char buf[2048]; int len=0;
	for(int i=0,end=strlen(str);i<end;i++) {
		if(str[i]=='@') len+=sprintf(buf+len,"%d",count);
		else buf[len++]=str[i];
	}
	buf[len]=0;
	printf("%s\n",buf);
}

void Blck(const char *str) {
	for(int n=0;n<count;n++) {
		char buf[2048]; int len=0;
		for(int i=0,end=strlen(str);i<end;i++) {
			if(str[i]=='#') len+=sprintf(buf+len,count>=10?"%2d":"%d",n);
			else buf[len++]=str[i];
		}
		buf[len]=0; Line(buf);
	}
}

void List(const char *str,const char *elem,const char *split=",") {
	char buf[4096]; int len=0;
	for(int i=0,end=strlen(str);i<end;i++) {
		if(str[i]=='#') {
			for(int n=0;n<count;n++) {
				for(int e=0,eend=strlen(elem);e<eend;e++) {
					if(elem[e]=='#') len+=sprintf(buf+len,count>=10?"%2d":"%d",n);
					else buf[len++]=elem[e];
				}
				if(n!=count-1) len+=sprintf(buf+len,"%s",split);
			}
		}
		else buf[len++]=str[i];
	}
	buf[len]=0; Line(buf);
}

int main(int argc,char **argv) {
	if(argc<2) return 0;
	count=atoi(argv[1]);

Line("template <class AccStruct> template <class Output>");
Line("void BIHTree<AccStruct>:: TraverseQuad@(const Vec3q *rOrigin,const Vec3q *tDir,floatq *out,i32x4 *object,TreeStats *tstats,int dirMask) const {");
List("	floatq maxD[@]={#};","out[#]");
Line(" ");
Line("		TreeStats stats;");
Line("		");
Line("		stats.TracingPacket(4*@);");
Line("");
Line("		Vec3q invDir[@]={");
Blck("			VInv(Vec3q(tDir[#].x+0.000000000001f,tDir[#].y+0.000000000001f,tDir[#].z+0.000000000001f)),");
Line("		};");
Line("		floatq tinv[3][@]={");
List("			{#},","invDir[#].x");
List("			{#},","invDir[#].y");
List("			{#} };","invDir[#].z");
Line("		floatq torig[3][@]={");
List("			{#},",  "rOrigin[#].x");
List("			{#},",  "rOrigin[#].y");
List("			{#} };","rOrigin[#].z");
Line("");
Line("		int dSign[3]; FillDSignArray(dirMask,dSign);");
List("		floatq minRet[@]={#},tMin[@],tMax[@];","maxD[#]");
Blck("		tMin[#]=ConstEpsilon<floatq>(); tMax[#]=Min(maxD[#],minRet[#]);");
Line("");
Line("		floatq fStackBegin[@*2*(maxLevel+2)],*fStack=fStackBegin;");
Line("		u32 nStackBegin[maxLevel+2],*nStack=nStackBegin;");
Line("");
Line("		{");
List("			Vec3q ttMin[@]={#};","(Vec3q(pMin)-rOrigin[#])*invDir[#]");
List("			Vec3q ttMax[@]={#};","(Vec3q(pMax)-rOrigin[#])*invDir[#]");
Line("");
List("			if(dSign[0]) {#;}","Swap(ttMin[#].x,ttMax[#].x)");
List("			if(dSign[1]) {#;}","Swap(ttMin[#].y,ttMax[#].y)");
List("			if(dSign[2]) {#;}","Swap(ttMin[#].z,ttMax[#].z)");
Line("");
Blck("			tMax[#]=Min(Min(ttMax[#].x,ttMax[#].y),tMax[#]);");
Blck("			tMax[#]=Min(ttMax[#].z,tMax[#]);");
Line("			");
Blck("			tMin[#]=Max(Max(ttMin[#].x,ttMin[#].y),tMin[#]);");
Blck("			tMin[#]=Max(ttMin[#].z,tMin[#]);");
Line("		}");
Line("		ObjectIdxBuffer<4> mailbox;");
Line("");
Line("		const BIHNode *node0=&nodes[0];");
Line("		int idx=0;");
Line("");
Line("		while(true) {");
Line("			stats.LoopIteration();");
Line("");
Line("			if(idx&BIHNode::leafMask) {");
Line("				idx&=BIHNode::idxMask;");
Line("");
Line("				if(!mailbox.Find(idx)) {");
Line("					mailbox.Insert(idx);");
Line("					stats.Intersection(@);");
Line("");
Line("					const Object &obj=objects[idx];");
Line("");
Line("					Vec3q tvec[@]; {");
Line("						Vec3q a(obj.a.x,obj.a.y,obj.a.z);");
Blck("						tvec[#]=rOrigin[#]-a;");
Line("					}");
Line("					floatq u[@],v[@]; {");
Line("						Vec3q ba(obj.ba.x,obj.ba.y,obj.ba.z),ca(obj.ca.x,obj.ca.y,obj.ca.z);");
Blck("						u[#]=tDir[#]|(ba^tvec[#]); v[#]=tDir[#]|(tvec[#]^ca);");
Line("					}");
Line("");
Line("					Vec3p nrm=obj.Nrm();");
Line("					floatq nrmLen=floatq( ((float*)&obj.ca)[3] );");

{ int tCount=count; for(count=0;count<tCount;count++) {
Line("					{");
Line("						floatq det=tDir[@]|nrm;");
Line("						f32x4b mask=Min(u[@],v[@])>=0.0f&&u[@]+v[@]<=det*nrmLen;");
Line("						if(ForAny(mask)) {");
Line("							floatq dist=Condition(mask,-(tvec[@]|nrm)/det,minRet[@]);");
Line("							mask=dist<minRet[@]&&dist>0.0f;");
Line("							minRet[@]=Condition(mask,Output::type==otShadow?0.00001f:dist,minRet[@]);");
Line("							if(Output::objectIndexes)");
Line("								object[@]=Condition(i32x4b(mask),i32x4(idx),object[@]);");
Line("							stats.IntersectPass();");
Line("						} else stats.IntersectFail();");
Line("					}");
 } }

Line("				}");
Line("");
Line("			POP_STACK:");
Line("				if(fStack==fStackBegin) break;");
Line("");
Line("				fStack-=2*@;");
Blck("				tMin[#]=fStack[#];");
Blck("				tMax[#]=Min(fStack[@+#],minRet[#]);");
Line("				--nStack;");
Line("				idx=*nStack;");
Line("				continue;");
Line("			}");
Line("");
Line("			const BIHNode *node=node0+(idx&BIHNode::idxMask);");
Line("			int axis=node->Axis();");
Line("			int nidx=dSign[axis];");
Line("			floatq near[@],far[@]; {");
Line("				floatq *start=torig[axis],*inv=tinv[axis];");
Line("");
Line("				float tnear=node->ClipLeft(),tfar=node->ClipRight();");
Line("				if(nidx) Swap(tnear,tfar);");
Line("");
Blck("				near[#]=Min( (floatq(tnear)-start[#])*inv[#], tMax[#]);");
Blck("				far [#]=Max( (floatq(tfar) -start[#])*inv[#], tMin[#]);");
Line("			}");
Line("");
List("			f32x4b test1=#;","tMin[#]>near[#]","&&");
List("			f32x4b test2=#;","tMax[#]<far [#]","&&");
Line("");
Line("			if(ForAll(test1)) {");
Line("				if(ForAll(test2)) goto POP_STACK;");
Line("");
Blck("				tMin[#]=far[#];");
Line("				idx=node->val[nidx^1];");
Line("				continue;");
Line("			}");
Line("			if(ForAll(test2)) {");
Line("				if(ForAll(test1)) goto POP_STACK;");
Line("");
Blck("				tMax[#]=near[#];");
Line("				idx=node->val[nidx];");
Line("				continue;");
Line("			}");
Line("");
Blck("			fStack[#]=far[#];");
Blck("			fStack[@+#]=tMax[#];");
Line("			fStack+=@*2;");
Line("");
Line("			*nStack=node->val[nidx^1];");
Line("			nStack++;");
Line("");
Blck("			tMax[#]=near[#];");
Line("			");
Line("			idx=node->val[nidx];");
Line("		}");
Line("");
Line("		if(tstats) tstats->Update(stats);");
Blck("		out[#]=minRet[#];");
Line("}");


	return 0;
}
