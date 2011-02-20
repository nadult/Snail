#include "scene.h"
#include "photons.h"

namespace {

	template <int size, class Shader>
	void Shade(const Shader &shader, Vec3q output[size]) {
		for(int q = 0; q < size; q++)
			output[q] = shader[q];
	}

	template <int size, class Shader, template <int> class Selector>
	void Shade(const Shader &shader, const Selector <size> &sel, Vec3q output[size]) {
		if(size >= 4) for(int t = 0; t < size / 4; t += 4) {
				if(!sel.Mask4(t)) continue;

				if(sel[t + 0]) output[t + 0] = Condition(sel.SSEMask(t + 0), shader[t + 0]);
				if(sel[t + 1]) output[t + 1] = Condition(sel.SSEMask(t + 1), shader[t + 1]);
				if(sel[t + 2]) output[t + 2] = Condition(sel.SSEMask(t + 2), shader[t + 2]);
				if(sel[t + 3]) output[t + 3] = Condition(sel.SSEMask(t + 3), shader[t + 3]);
			}
		for(int q = (size / 4) * 4; q < size; q++)
			if(sel[q]) output[q] = Condition(sel.SSEMask(q), shader[q]);
	}

	template <class Context>
	void InitializationShader(Context &c, uint i, const typename Context::real &maxDist) {
		Vec3q ambient = Vec3f(0.4, 0.4, 0.4);

		c.distance[i] = maxDist;
		c.color[i]    = f32x4(0.0f);
		c.light[i]    = ambient;
		c.objId[i]    = c.elementId[i] = i32x4(0);
	}

	template <class Context>
	void SimpleLightingShader(Context &c, uint i) {
		c.color[i] = c.RayDir(i) | c.normal[i];
	//	c.color[i]*=c.color[i];
	//	c.color[i]*=c.color[i];
	}

	template <class Context>
	void ReflectionShader(Context &c, uint i) {
		typedef typename Context::real    real;
		Vec3q refl = Reflect(c.RayDir(i), c.normal[i]);

		c.color[i].x = Condition(refl.x > real(0.0f), real(8.0f / 12.0f), real(2.0f / 12.0f));
		c.color[i].y = Condition(refl.y > real(0.0f), real(8.0f / 12.0f), real(2.0f / 12.0f));
		c.color[i].z = Condition(refl.z > real(0.0f), real(8.0f / 12.0f), real(2.0f / 12.0f));
	}

	template <class Context>
	void RayDirectionShader(Context &c, uint i) {
		Vec3q dir = c.RayDir(i);

		c.color[i].x = Condition(dir.x < floatq(0.0f), floatq(0.5f), floatq(1.0f));
		c.color[i].y = Condition(dir.y < floatq(0.0f), floatq(0.5f), floatq(1.0f));
		c.color[i].z = Condition(dir.z < floatq(0.0f), floatq(0.5f), floatq(1.0f));
	}

	struct StatsShader
	{
		StatsShader(const TreeStats &tStats, int size) : stats(tStats), size(size) { }
		Vec3q operator[](int) const {
			return Vec3q(log(stats.timers[0]) * 0.05f, 0.0f, 0.0f);

			return Vec3q(
				   float(stats.GetIntersects()) * (0.002f / size),
				   float(stats.GetLoopIters()) * (0.02f / size),
				   float(stats.GetSkips() * 0.25f));
		}

		const TreeStats &stats;
		int size;
	};

	struct DistanceShader
	{
		DistanceShader(const floatq *dist) : distance(dist) { }
		Vec3q operator[](int q) const {
			floatq idist = Inv(distance[q]);

			return Vec3q(Abs(idist * floatq(1.0f)), Abs(idist * floatq(0.1f)), Abs(idist * floatq(0.01f)));
		}

		const floatq *distance;
	};

}

template <class AccStruct> template <bool sharedOrigin, bool hasMask>
const TreeStats Scene<AccStruct>::RayTrace(const RayGroup <sharedOrigin, hasMask> &rays, Cache &cache,
		Vec3q *__restrict__ outColor) const {
	int size = rays.Size();

	enum {
		flags = sharedOrigin ? isct::fShOrig | isct::fPrimary : 0,
		blockSize = 4,
	};

	TreeStats stats;

	static_assert(int(blockSize) == int(shading::blockSize), "Block sizes should be equal");

	const floatq maxDist = 1.0f / 0.0f;

	floatq tDistance[size];
	i32x4 tObject[size], tElement[size];
	Vec2q barycentric[size];

	for(int q = 0; q < size; q++) {
		tDistance[q] = Condition(rays.SSEMask(q), maxDist, -constant::inf);
		tObject[q] = 0;
	}
	for(int q = 0; q < size; q++)
		stats.TracingRays(CountMaskBits(rays.Mask(q)));

	Context<sharedOrigin, hasMask> tc(rays, tDistance, tObject, tElement, barycentric, &stats);
	geometry.TraversePrimary(tc);

	char selectorData[size + 4], reflSelData[size + 4], transSelData[size + 4];
	RaySelector selector(selectorData, size), reflSel(reflSelData, size), transSel(transSelData, size);
	for(int n = 0; n < size; n++)
		selector[n] = rays.Mask(n);
	reflSel.Clear(); transSel.Clear();

	if(gVals[1]) {     //very simple shading
		for(int q = 0; q < size; q++) {
			floatq dist  = (Condition(tDistance[q] > maxDist, 0.0f, Inv(tDistance[q])));
			floatq dist2 = dist * 250.0f;
			floatq dist1 = dist * 20.0f;
			floatq dist3 = dist * 2.0f;
			outColor[q] = Vec3q(dist1, dist2, dist3);
		}
		return stats;
	}

	Vec3q minPos(1.0f / 0.0f, 1.0f / 0.0f, 1.0f / 0.0f);
	Vec3q maxPos(-1.0f / 0.0f, -1.0f / 0.0f, -1.0f / 0.0f);

	shading::Sample samples[size];
//	int matCount = materials.size();

	if(gVals[6]) { // full shading
		for(uint b = 0, nBlocks = size / blockSize; b < nBlocks; b++) {
			uint b4 = b << 2;

			f32x4b mask[blockSize];
			i32x4  matId[blockSize], object[blockSize], element[blockSize];

			for(int q = 0; q < blockSize; q++) {
				int tq = b4 + q;

				shading::Sample &s = samples[tq];

				mask[q] = tDistance[tq] < maxDist && selector.SSEMask(tq);
				selector[tq] = ForWhich(mask[q]);
				i32x4b imask(mask[q]);

				object[q]  = Condition(imask, tObject[tq], i32x4(0));
				element[q] = AccStruct::isctFlags & isct::fElement ? Condition(imask, tElement[tq], i32x4(0)) : i32x4(0);
				s.position = rays.Dir(tq) * tDistance[tq] + rays.Origin(tq);
				minPos     = Condition(mask[q], VMin(minPos, s.position), minPos);
				maxPos     = Condition(mask[q], VMax(maxPos, s.position), maxPos);
 
				s.diffuse = s.specular = Vec3q(0.0f, 0.0f, 0.0f);
				matId[q]  = ~0;
			}

			int mask4 = selector.Mask4(b);
			if(!mask4) continue;

			shading::Sample *s = samples + b4;
			int obj0 = object[0][0], elem0 = element[0][0];

			// 4x4 full, single triangle
			if(mask4 == 0xf0f0f0f &&
			   ForAll(((object[0] == object[1] && element[0] == element[1]) &&
					   (object[2] == object[3] && element[2] == element[3])) &&
					  ((object[0] == object[2] && element[0] == element[2]) &&
					   (object[0] == obj0 && element[0] == elem0)))) {
				const ShTriangle shTri = geometry.GetSElement(obj0, elem0);
				int tMatId = geometry.GetMaterialId(shTri.MatId(), obj0);

				const bool flatNormals = shTri.FlatNormals();
				const shading::Material *mat = tMatId == ~0? &defaultMat : &*materials[tMatId];
				const bool computeTexCoords = mat->flags & shading::Material::fTexCoords;
				reflSel.Mask4(b) = mat->flags & shading::Material::fReflection ? 0x0f0f0f0f : 0;
				transSel.Mask4(b) = mat->flags & shading::Material::fTransparency ? 0x0f0f0f0f : 0;

				if(!computeTexCoords) {
					if(flatNormals) {
						Vec3q nrm0(shTri.nrm[0]);
						s[0].normal = s[1].normal = s[2].normal = s[3].normal = nrm0;
					}
					else {
						Vec3f nrm0 = shTri.nrm[0];
						Vec3q nrm1(shTri.nrm[1]);
						Vec3q nrm2(shTri.nrm[2]);

						for(int q = 0; q < 4; q++) {
							const Vec2q bar = barycentric[b4 + q];
							s[q].normal = Vec3q(nrm0) + (Vec3q(nrm1) * bar.x + Vec3q(nrm2) * bar.y);
						}
					}
				}
				else {
					Vec2q uv0  = Vec2q(shTri.uv[0].x, shTri.uv[0].y);
					Vec2q uv1  = Vec2q(shTri.uv[1].x, shTri.uv[1].y);
					Vec2q uv2  = Vec2q(shTri.uv[2].x, shTri.uv[2].y);
					Vec3f nrm0 = shTri.nrm[0];
					Vec3f nrm1 = shTri.nrm[1];
					Vec3f nrm2 = shTri.nrm[2];

					for(int q = 0; q < 4; q++) {
						const Vec2q bar = barycentric[b4 + q];
						s[q].texCoord = uv0 + uv1 * bar.x + uv2 * bar.y;
						Broadcast(Maximize(s[q].texCoord) - Minimize(s[q].texCoord), s[q].texDiff);
						s[q].normal = Vec3q(nrm0) + Vec3q(nrm1) * bar.x + Vec3q(nrm2) * bar.y;
					}
				}

				mat->Shade(s, RayGroup<sharedOrigin, hasMask>(rays, b4, 4), cache.samplingCache);
			}
			else {
				for(uint q = 0; q < blockSize; q++) {
					uint tq = b4 + q;
					if(!selector[tq]) continue;

					shading::Sample &s = samples[tq];

					i32x4b imask(mask[q]);
					int    invBitMask = ~ForWhich(imask);

					int obj0 = object[q][0], elem0 = element[q][0];
					if(EXPECT_TAKEN( i32x4(imask)[0] )) {
						const ShTriangle shTri = geometry.GetSElement(obj0, elem0);

						int tMatId = geometry.GetMaterialId(shTri.MatId(), obj0);
						matId[q] = Condition(imask, i32x4(tMatId), matId[q]);

						const Vec2q bar = barycentric[tq];

						s.texCoord = Vec2q(shTri.uv[0].x, shTri.uv[0].y) +
									 Vec2q(shTri.uv[1].x, shTri.uv[1].y) * bar.x +
									 Vec2q(shTri.uv[2].x, shTri.uv[2].y) * bar.y;

						s.normal = Vec3q(shTri.nrm[0]) + Vec3q(shTri.nrm[1]) * bar.x + Vec3q(shTri.nrm[2]) * bar.y;
					}

					if(AccStruct::isctFlags & isct::fElement ?
							ForAny(object[q] != obj0 || element[q] != elem0) : ForAny(object[q] != obj0)) {

						float texCoordX[4], texCoordY[4];
						Vec3f normal[4];
						Convert(s.texCoord.x, texCoordX);
						Convert(s.texCoord.y, texCoordY);
						Convert(s.normal, normal);

						for(int k = 1; k < 4; k++) {
							int obj = object[q][k], elem = element[q][k];
							if((invBitMask & (1 << k)) || (obj == obj0 && elem == elem0)) continue;

							const ShTriangle shTri = geometry.GetSElement(obj, elem);

							int tMatId = geometry.GetMaterialId(shTri.MatId(), obj);
							matId[q][k] = tMatId;

							const Vec2q bar = barycentric[tq];

							Vec2q tex = Vec2q(shTri.uv[0].x, shTri.uv[0].y) +
										Vec2q(shTri.uv[1].x, shTri.uv[1].y) * bar.x +
										Vec2q(shTri.uv[2].x, shTri.uv[2].y) * bar.y;

							Vec2f diff = Maximize(tex) - Minimize(tex);
							texCoordX[k] = tex.x[k];
							texCoordY[k] = tex.y[k];
							Vec3f nrm = shTri.nrm[0] + shTri.nrm[1] * bar.x[k] + shTri.nrm[2] * bar.y[k];

							normal[k].x = nrm.x;
							normal[k].y = nrm.y;
							normal[k].z = nrm.z;
						}

						Convert(texCoordX, s.texCoord.x);
						Convert(texCoordY, s.texCoord.y);
						Convert(normal, s.normal);
					}
				}
				
				//todo: sa problemy na krawedziach trojkatow o roznych materialach (a nawet tych samych materialow)
				for(int q = 0; q < blockSize; q++) {
					shading::Sample &s = samples[q + b4];
					Vec2f diff = Maximize(s.texCoord) - Minimize(s.texCoord);
					s.texDiff = /*ForAll(object[q] == object[q][0]) && gVals[1]? Vec2q(diff) :*/ Vec2q(0.0f, 0.0f);
				}

				int matId0 = matId[0][0];

				if(mask4 == 0x0f0f0f0f && 
					ForAll( ( matId[0] == matId[1] && matId[2] == matId[3] ) &&
							( matId[0] == matId[2] && matId[0] == matId0   ) )) {
					const shading::Material *mat = matId0 == ~0? &defaultMat : &*materials[matId0];
					reflSel.Mask4(b) = mat->flags & shading::Material::fReflection ? 0x0f0f0f0f : 0;
					transSel.Mask4(b) = mat->flags & shading::Material::fTransparency ? 0x0f0f0f0f : 0;

					mat->Shade(s, RayGroup<sharedOrigin, 0>(rays, b4, 4), cache.samplingCache);
				}
				else {
					static_assert(blockSize == 4, "4 is nice");
					i32x4 matIds[blockSize] = { i32x4(~0), i32x4(~0), i32x4(~0), i32x4(~0) };
					bool defaultUsed = 0;

					uint matIdCount = 0;
					for(int q = 0; q < blockSize; q++) {
						const i32x4 tMatId = matId[q];

						for(int k = 0; k < 4; k++) {
							int id = tMatId[k];
							defaultUsed |= id == ~0;

							i32x4 kMatId(id);
							if( (selector[b4 + q] & (1 << k)) && 
								ForAll(	matIds[0] != kMatId && matIds[1] != kMatId &&
										matIds[2] != kMatId && matIds[3] != kMatId ) ) {
								matIds[matIdCount >> 2][matIdCount & 3] = id;
								matIdCount++;
							}
						}
					}
					if(defaultUsed) matIdCount++;

					for(uint n = 0; n < matIdCount; n++) {
						uint id = matIds[n >> 2][n & 3];

						char mask[4];
						i32x4 id4(id);
						mask[0] = ForWhich(matId[0] == id4) & selector[b4 + 0];
						mask[1] = ForWhich(matId[1] == id4) & selector[b4 + 1];
						mask[2] = ForWhich(matId[2] == id4) & selector[b4 + 2];
						mask[3] = ForWhich(matId[3] == id4) & selector[b4 + 3];
						const shading::Material *mat = id == ~0? &defaultMat : &*materials[id];
						if(mat->flags & shading::Material::fReflection)
							for(int q = 0; q < 4; q++)
								reflSel[b4 + q] = mask[q];
						if(mat->flags & shading::Material::fTransparency)
							for(int q = 0; q < 4; q++)
								transSel[b4 + q] = mask[q];

						mat->Shade(samples + b4, RayGroup<sharedOrigin, 1>( rays.OriginPtr() +
								(sharedOrigin? b4 : 0), rays.DirPtr() + b4, rays.IDirPtr() + b4, 4, mask),
								cache.samplingCache);
					}
				}
			}
		}
	}
	else { //simple shading
		for(uint b = 0, nBlocks = size / blockSize; b < nBlocks; b++) {
			uint b4 = b << 2;

			f32x4b mask[blockSize];
			i32x4  matId[blockSize], object[blockSize], element[blockSize];

			for(int q = 0; q < blockSize; q++) {
				int tq = b4 + q;

				shading::Sample &s = samples[tq];

				mask[q] = tDistance[tq] < maxDist && selector.SSEMask(tq);
				selector[tq] = ForWhich(mask[q]);
				i32x4b imask(mask[q]);

				object[q]  = Condition(imask, tObject[tq], i32x4(0));
				element[q] = AccStruct::isctFlags & isct::fElement ? Condition(imask, tElement[tq], i32x4(0)) : i32x4(0);
				s.position = rays.Dir(tq) * tDistance[tq] + rays.Origin(tq);
				minPos     = Condition(mask[q], VMin(minPos, s.position), minPos);
				maxPos     = Condition(mask[q], VMax(maxPos, s.position), maxPos);

				s.diffuse = s.specular = Vec3q(0.0f, 0.0f, 0.0f);
				matId[q]  = ~0;
			}

			int mask4 = selector.Mask4(b);
			if(!mask4) continue;

			shading::Sample *s = samples + b4;
			int obj0 = object[0][0], elem0 = element[0][0];

			// 4x4 full, single triangle
			if(mask4 == 0xf0f0f0f &&
			   ForAll(((object[0] == object[1] && element[0] == element[1]) &&
					   (object[2] == object[3] && element[2] == element[3])) &&
					  ((object[0] == object[2] && element[0] == element[2]) &&
					   (object[0] == obj0 && element[0] == elem0)))) {
				Vec3q nrm0(geometry.GetNormal(obj0, elem0));
				s[0].normal = s[1].normal = s[2].normal = s[3].normal = nrm0;
				defaultMat.Shade(s, RayGroup<sharedOrigin, hasMask>(rays, b4, 4), cache.samplingCache);
			}
			else {
				for(uint q = 0; q < blockSize; q++) {
					uint tq = b4 + q;
					if(!selector[tq]) continue;

					shading::Sample &s = samples[tq];

					i32x4b imask(mask[q]);
					int    invBitMask = ~ForWhich(imask);

					int obj0 = object[q][0], elem0 = element[q][0];
					if(EXPECT_TAKEN( i32x4(imask)[0] ))
						s.normal = Vec3q(geometry.GetNormal(obj0, elem0));

					if(AccStruct::isctFlags & isct::fElement ?
							ForAny(object[q] != obj0 || element[q] != elem0) : ForAny(object[q] != obj0)) {

						Vec3f normal[4];
						Convert(s.normal, normal);

						for(int k = 1; k < 4; k++) {
							int obj = object[q][k], elem = element[q][k];
							if((invBitMask & (1 << k)) || (obj == obj0 && elem == elem0)) continue;

							Vec3f nrm = geometry.GetNormal(obj, elem);
							normal[k].x = nrm.x;
							normal[k].y = nrm.y;
							normal[k].z = nrm.z;
						}

						Convert(normal, s.normal);
					}
				}
				
				if(mask4 == 0x0f0f0f0f)
					defaultMat.Shade(s, RayGroup<sharedOrigin, 0>(rays, b4, 4), cache.samplingCache);
				else {
					static_assert(blockSize == 4, "4 is nice");
				
					char mask[4];
					mask[0] = selector[b4 + 0];
					mask[1] = selector[b4 + 1];
					mask[2] = selector[b4 + 2];
					mask[3] = selector[b4 + 3];

					defaultMat.Shade(samples + b4, RayGroup<sharedOrigin, 1>( rays.OriginPtr() +
							(sharedOrigin? b4 : 0), rays.DirPtr() + b4, rays.IDirPtr() + b4, 4, mask),
							cache.samplingCache);
				}
			}
		}
	}

	if(/*reflSel.Any() &&*/ gVals[7] && cache.reflections < 1) {
		//TODO: przywrocic obsluge wlaczania odbic dla wybranych materialow
		reflSel = selector;
		cache.reflections++;
		Vec3q reflColor[size];
		stats += TraceReflection(reflSel, rays.DirPtr(), samples, cache, reflColor);
		cache.reflections--;

		for(int q = 0; q < size; q++) {
			samples[q].diffuse = Condition(reflSel.SSEMask(q), samples[q].diffuse +
					(reflColor[q] - samples[q].diffuse) * floatq(0.3f), samples[q].diffuse);
		}
	}
	if(0 && transSel.Any()) {
		Vec3q transColor[size];
		for(int q = 0; q < size; q++)
			transSel[q] &= ForWhich(samples[q].opacity < 1.0f);

		if(transSel.Any()) {
			cache.transp++;
			stats += TraceTransparency(transSel, rays, tDistance, transColor, cache);
			cache.transp--;

			for(int q = 0; q < size; q++)
				samples[q].diffuse = Condition(transSel.SSEMask(q),
						VLerp(transColor[q], samples[q].diffuse, samples[q].opacity), samples[q].diffuse);
		}
	}

	Vec3q lDiffuse[size], lSpecular[size];
	if(lights.size()) {
		Vec3q dif(ambientLight), spec(0.0f, 0.0f, 0.0f);
		for(int q = 0; q < size; q++) {
			lDiffuse[q]  = dif;
			lSpecular[q] = spec;
		}
	}

	if(photons) {
		stats.timers[0] = GatherPhotons(*photonNodes, *photons, samples, size, 3.0, geometry.GetBBox());
		for(unsigned q = 0; q < size; q++)
			samples[q].diffuse += samples[q].temp1;
		
	}

	if(0) {
		int   nLights = lights.size();
		Vec3f tMinPos(Minimize(minPos)), tMaxPos(Maximize(maxPos));
		for(int n = 0; n < nLights; n++) {
			const Light &light = lights[n];
			if(BoxPointDistanceSq(BBox(tMinPos, tMaxPos), light.pos) > light.radSq)
				continue;

			stats += TraceLight(selector, samples, lDiffuse, lSpecular, n);
		}
	}

	if(lights.size()) {
		Vec3q zero(0.0f, 0.0f, 0.0f), one(1.0f, 1.0f, 1.0f);
		for(int q = 0; q < size; q++) {
			outColor[q] = samples[q].diffuse * lDiffuse[q] + samples[q].specular * lSpecular[q];
		}
	}
	else for(int q = 0; q < size; q++)
		outColor[q] = samples[q].diffuse;

	if(gVals[5] && (flags & isct::fPrimary)) {
		Vec3q col = StatsShader(stats, size)[0];
		for(int q = 0; q < size; q++)
			outColor[q] = col;
	}

	return stats;
}

template <class AccStruct>
const TreeStats Scene <AccStruct>::TraceLight(RaySelector inputSel, const shading::Sample *samples,
		Vec3q *__restrict__ diffuse, Vec3q *__restrict__ specular, int idx) const {
	int size = inputSel.Size();
	enum { shadows = 1 };

	TreeStats stats;

	const Light &light = lights[idx];
	Vec3f lightPos = light.pos;

	Vec3q fromLight[size], idir[size];
	floatq distance[size], tDistance[size];
	floatq dot[size];

	for(int q = 0; q < size; q++) {
		if(!inputSel[q]) {
			tDistance[q] = -constant::inf;
			continue;
		}
		const shading::Sample &s = samples[q];

		Vec3q  lightVec = (s.position - Vec3q(lightPos));
		f32x4b close = LengthSq(lightVec) < 0.0001f;
		lightVec = Condition(close, Vec3q(0.0f, 1.0f, 0.0f), lightVec);

		distance[q] = Sqrt(lightVec | lightVec);
		fromLight[q] = lightVec * Inv(distance[q]);
		idir[q] = SafeInv(fromLight[q]);

		dot[q] = s.normal | fromLight[q];
		f32x4b mask = inputSel.SSEMask(q) && dot[q] > 0.0f;

		tDistance[q] = Condition(mask, distance[q] * 0.9999f, -constant::inf);
		stats.TracingRays(CountMaskBits(ForWhich(mask)));
	}

	if(shadows) {
		Vec3q lPos(lightPos.x, lightPos.y, lightPos.z);
		ShadowContext c(RayGroup<1, 0>(&lPos, fromLight, idir, size), tDistance, &stats);
		geometry.TraverseShadow(c);

		// All pixels masked or in shadow?
		f32x4 tdmax(-constant::inf);
		int q = 0;
		for(; q + 3 < size; q += 4)
			tdmax = Max(tdmax,	Max( Max(tDistance[q + 0], tDistance[q + 1]),
									 Max(tDistance[q + 2], tDistance[q + 3])) );
		for(;q < size; q++)
			tdmax = Max(tdmax, tDistance[q]);
		if(ForAll(tdmax < 0.0f))
			return stats;
	}
	
	Vec3q lPos(lightPos.x, lightPos.y, lightPos.z);
	Vec3q lColor(light.color.x, light.color.y, light.color.z);

	for(int q = 0; q < size; q++) {
		f32x4  dist = distance[q];
		f32x4b msk  = tDistance[q] > 0.0f;

		f32x4 atten = dist * light.iRadius;
		atten = Max(f32x4(0.0f), ((floatq(1.0f) - atten) * 0.2f + FastInv(f32x4(16.0f) * atten * atten)) -
					f32x4(0.0625f));

		f32x4 diffMul = dot[q] * atten;
		f32x4 specMul = dot[q];
		specMul *= specMul;
		specMul *= specMul;
		specMul *= specMul;
		specMul *= specMul;
		specMul *= atten;

		diffuse [q] += Condition(msk, lColor * diffMul);
		specular[q] += Condition(msk, lColor * specMul);
	}

	return stats;
}


template <class AccStruct>
const TreeStats Scene<AccStruct>::TraceReflection(RaySelector selector, const Vec3q *dir, const shading::Sample *samples,
		Cache &cache, Vec3q *__restrict__ outColor) const {
	int size = selector.Size();
	Vec3q reflDir[size], reflOrig[size], idir[size];

	for(int q = 0; q < size; q++) {
		reflDir[q] = Reflect(dir[q], samples[q].normal);
		reflOrig[q] = samples[q].position + reflDir[q] * floatq(0.001f);
		idir[q] = SafeInv(reflDir[q]);
	}

	return selector.All()?
		RayTrace(RayGroup<0, 0>(reflOrig, reflDir, idir, size), cache, outColor) :
		RayTrace(RayGroup<0, 1>(reflOrig, reflDir, idir, size, &selector[0]), cache, outColor);
}
	
template <class AccStruct>
template <bool sharedOrigin, bool hasMask>
const TreeStats Scene<AccStruct>::TraceTransparency(RaySelector selector,
		const RayGroup<sharedOrigin, hasMask> &rays, const floatq *distance, Vec3q *__restrict__ out,
		Cache &cache) const {

	const int size = rays.Size();
	Vec3q origin[size];
	for(int q = 0; q < size; q++) //TODO: robust epsilons
		origin[q] = rays.Dir(q) * (distance[q] + 0.0001) + rays.Origin(q);

	return selector.All()?
		RayTrace(RayGroup<0, 0>(origin, rays.DirPtr(), rays.IDirPtr(), size), cache, out) :
		RayTrace(RayGroup<0, 1>(origin, rays.DirPtr(), rays.IDirPtr(), size, &selector[0]), cache, out);
}

#include "bvh/tree.h"
#include "dbvh/tree.h"

template const TreeStats Scene<BVH>::TraceLight(RaySelector, const shading::Sample*,
		Vec3q *__restrict__, Vec3q *__restrict__, int) const;

template const TreeStats Scene<DBVH>::TraceLight(RaySelector, const shading::Sample*,
		Vec3q *__restrict__, Vec3q *__restrict__, int) const;

template const TreeStats Scene<BVH>::TraceReflection(RaySelector, const Vec3q*,
		const shading::Sample*, Cache&, Vec3q*__restrict__) const;

template const TreeStats Scene<DBVH>::TraceReflection(RaySelector, const Vec3q*,
		const shading::Sample*, Cache&, Vec3q*__restrict__) const;


template const TreeStats Scene<BVH>::RayTrace(const RayGroup <0, 0>&, Cache&, Vec3q*__restrict__) const;
template const TreeStats Scene<BVH>::RayTrace(const RayGroup <0, 1>&, Cache&, Vec3q*__restrict__) const;
template const TreeStats Scene<BVH>::RayTrace(const RayGroup <1, 0>&, Cache&, Vec3q*__restrict__) const;
//template const TreeStats Scene<BVH>::RayTrace(const RayGroup <1, 1>&, Cache&, Vec3q*__restrict__) const;

template const TreeStats Scene<DBVH>::RayTrace(const RayGroup <0, 0>&, Cache&, Vec3q*__restrict__) const;
template const TreeStats Scene<DBVH>::RayTrace(const RayGroup <0, 1>&, Cache&, Vec3q*__restrict__) const;
template const TreeStats Scene<DBVH>::RayTrace(const RayGroup <1, 0>&, Cache&, Vec3q*__restrict__) const;
//template const TreeStats Scene<DBVH>::RayTrace(const RayGroup <1, 1>&, Cache&, Vec3q*__restrict__) const;
