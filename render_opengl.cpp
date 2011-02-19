#include "render_opengl.h"
#include "bvh/tree.h"
#include "camera.h"
#define GL_GLEXT_PROTOTYPES 1
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glu.h>
#include "shading/material.h"

OGLRenderer::OGLRenderer(const Scene<BVH> &scene) {
	unsigned int temp[3];
	glGenBuffers(3, temp);
	InputAssert(glGetError() == GL_NO_ERROR);

	posBuffer = temp[0];
	uvBuffer = temp[1];
	nrmBuffer = temp[2];
	photonBuffer = 0;
	photonColorsBuffer = 0;
	photonBufferSize = 0;

	std::cout << "Loading mesh to GPU...\n";

	const ATriVector &tris = scene.geometry.tris;
	{
		glBindBuffer(GL_ARRAY_BUFFER, posBuffer);
		vector<Vec3f> positions(tris.size() * 3);
		for(size_t n = 0; n < tris.size(); n++) {
			positions[n * 3 + 0] = tris[n].P1();
			positions[n * 3 + 1] = tris[n].P2();
			positions[n * 3 + 2] = tris[n].P3();
		}
		glBufferData(GL_ARRAY_BUFFER, tris.size() * 3 * 3 * 4, &positions[0], GL_STATIC_DRAW);
		InputAssert(glGetError() == GL_NO_ERROR);
	}

	const AShTriVector &shTris = scene.geometry.shTris;

	useUvs = false;
	for(size_t n = 0; n < scene.materials.size(); n++)
		if(scene.materials[n]->flags & shading::Material::fTexCoords)
			useUvs = true;

	if(shTris.size() == tris.size() && useUvs) {
		glBindBuffer(GL_ARRAY_BUFFER, uvBuffer);
		vector<Vec2f> uvs(shTris.size() * 3);
		for(size_t n = 0; n < tris.size(); n++) {
			uvs[n * 3 + 0] = shTris[n].uv[0];
			uvs[n * 3 + 1] = shTris[n].uv[1];
			uvs[n * 3 + 2] = shTris[n].uv[2];
		}
		glBufferData(GL_ARRAY_BUFFER, shTris.size() * 3 * 2 * 4, &uvs[0], GL_STATIC_DRAW);
		InputAssert(glGetError() == GL_NO_ERROR);
	}
	else useUvs = false;

	{
		useNormals = true;
		glBindBuffer(GL_ARRAY_BUFFER, nrmBuffer);
		vector<Vec3f> normals(shTris.size() * 3);

		if(shTris.size() == tris.size()) {
			for(size_t n = 0; n < tris.size(); n++) {
				normals[n * 3 + 0] = shTris[n].nrm[0];
				normals[n * 3 + 1] = shTris[n].nrm[1] + shTris[n].nrm[0];
				normals[n * 3 + 2] = shTris[n].nrm[2] + shTris[n].nrm[0];
			}
		}
		else {
			for(size_t n = 0; n < tris.size(); n++)
				normals[n * 3 + 0] = normals[n * 3 + 1] = normals[n * 3 + 2] =  Vec3f(tris[n].plane);
		}
		glBufferData(GL_ARRAY_BUFFER, shTris.size() * 3 * 3 * 4, &normals[0], GL_STATIC_DRAW);
		InputAssert(glGetError() == GL_NO_ERROR);
	}

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	triCount = tris.size();
	bbox = scene.geometry.GetBBox();

	//TODO: zrobic porzadna obsluge wyjatkow
	try {
		InitShaders();
	}
	catch(...) {
		glDeleteBuffers(1, &posBuffer);
		glDeleteBuffers(1, &uvBuffer);
		glDeleteBuffers(1, &nrmBuffer);
		throw;
	}
}

void OGLRenderer::InitShaders() {
	unsigned v = glCreateShader(GL_VERTEX_SHADER);
	unsigned f = glCreateShader(GL_FRAGMENT_SHADER);

	const char *fs = "varying vec4 position, normal; void main() {\n"
//					"float col = max(normal.z, 0.0) * 0.2; gl_FragColor = vec4(col, col, col, 1); }";
					"gl_FragColor = vec4(abs(normal.xyz), 1) * 0.1; }";
	const char *vs = "varying vec4 position, normal; void main() {\n"
	   				"normal = vec4(gl_Normal, 0.0);//(gl_ModelViewProjectionMatrix * vec4(gl_Normal, 0.0));\n"
				   	"gl_Position = (gl_ModelViewProjectionMatrix * gl_Vertex); }";

	glShaderSource(v, 1, &vs, 0);
	glShaderSource(f, 1, &fs, 0);

	glCompileShader(v);
	glCompileShader(f);

	program = glCreateProgram();

	glAttachShader(program, f);
	glAttachShader(program, v);

	glLinkProgram(program);

	{
		char buf[4096];
		glGetProgramInfoLog(program, sizeof(buf), 0, buf);
		std::cout << buf << '\n';
	}
	InputAssert(glGetError() == GL_NO_ERROR);
}

OGLRenderer::~OGLRenderer() {
	glDeleteBuffers(1, &posBuffer);
	glDeleteBuffers(1, &uvBuffer);
	glDeleteBuffers(1, &nrmBuffer);
	if(photonBuffer)
		glDeleteBuffers(1, &photonBuffer);
	if(photonColorsBuffer)
		glDeleteBuffers(1, &photonColorsBuffer);
	glDeleteProgram(program);
}

void OGLRenderer::BeginDrawing(const Camera &cam, float fov, float aspect, bool clearColor) const {
	glClear((clearColor? GL_COLOR_BUFFER_BIT : 0) | GL_DEPTH_BUFFER_BIT);

	glDisable(GL_TEXTURE_2D);

	Vec3f size = bbox.Size();
	float tsize = size.x + size.y + size.z;

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	gluPerspective(fov, aspect, tsize * 0.01f, tsize * 10.0f);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	Vec3f eye = cam.pos, center = cam.pos + cam.front * 10.0f, up = cam.up;
	gluLookAt(eye.x, eye.y, eye.z, center.x, center.y, center.z, up.x, up.y, up.z);
}

void OGLRenderer::FinishDrawing() const {
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

void OGLRenderer::Draw() const {
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_FRONT);

	glUseProgram(program);

		glBindBuffer(GL_ARRAY_BUFFER, posBuffer);
		glVertexPointer(3, GL_FLOAT, 0, 0);
		glEnableClientState(GL_VERTEX_ARRAY);
		
		if(useUvs) {
			glBindBuffer(GL_ARRAY_BUFFER, uvBuffer);
			glTexCoordPointer(2, GL_FLOAT, 0, 0);
			glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		}
		else glDisableClientState(GL_TEXTURE_COORD_ARRAY);

		if(useNormals) {
			glBindBuffer(GL_ARRAY_BUFFER, nrmBuffer);
			glNormalPointer(GL_FLOAT, 0, 0);
			glEnableClientState(GL_NORMAL_ARRAY);
		}
		else glDisableClientState(GL_NORMAL_ARRAY);

		glDrawArrays(GL_TRIANGLES, 0, triCount * 3);

	glUseProgram(0);

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
}
	

void OGLRenderer::DrawPhotons(const vector<Photon> &photons, bool update) {
	if(update) {
		if(!photonBuffer || photonBufferSize < photons.size()) {
			if(photonBuffer) {
				glDeleteBuffers(1, &photonBuffer);
				glDeleteBuffers(1, &photonColorsBuffer);
			}
			glGenBuffers(1, &photonBuffer);
			glGenBuffers(1, &photonColorsBuffer);

			glBindBuffer(GL_ARRAY_BUFFER, photonBuffer);
			glBufferData(GL_ARRAY_BUFFER, photons.size() * 4 * 3, 0, GL_DYNAMIC_DRAW);

			glBindBuffer(GL_ARRAY_BUFFER, photonColorsBuffer);
			glBufferData(GL_ARRAY_BUFFER, photons.size() * 4, 0, GL_DYNAMIC_DRAW);
			photonBufferSize = photons.size();
		}	
		const unsigned step = 4096;

		glBindBuffer(GL_ARRAY_BUFFER, photonBuffer);
		for(unsigned n = 0; n < photons.size(); n += step) {
			Vec3f pbuf[step];
			unsigned count = Min(step, photons.size() - n);
			for(unsigned i = 0; i < count; i++)
				pbuf[i] = photons[n + i].position;
			glBufferSubData(GL_ARRAY_BUFFER, n * 4 * 3, count * 4 * 3, pbuf);
		}

		glBindBuffer(GL_ARRAY_BUFFER, photonColorsBuffer);
		for(unsigned n = 0; n < photons.size(); n += step) {
			unsigned cbuf[step];
			unsigned count = Min(step, photons.size() - n);
			for(unsigned i = 0; i < count; i++) {
				const Photon &photon = photons[n + i];
				cbuf[i] = *((u32*)photon.color);
			}
			glBufferSubData(GL_ARRAY_BUFFER, n * 4, count * 4, cbuf);
		}
	}

	if(!photonBufferSize)
		return;

	glEnable(GL_DEPTH_TEST);

	glPointSize(3.0f);

	glBindBuffer(GL_ARRAY_BUFFER, photonBuffer);
	glVertexPointer(3, GL_FLOAT, 0, 0);
	glEnableClientState(GL_VERTEX_ARRAY);
		
	glBindBuffer(GL_ARRAY_BUFFER, photonColorsBuffer);
	glColorPointer(4, GL_UNSIGNED_BYTE, 0, 0);
	glEnableClientState(GL_COLOR_ARRAY);

	glDrawArrays(GL_POINTS, 0, photonBufferSize);
		
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glPointSize(1.0f);
	glDisable(GL_DEPTH_TEST);
}
