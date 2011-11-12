#include "pch.h"
#include "vrender_opengl.h"
#define GL_GLEXT_PROTOTYPES 1
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glu.h>
#include <cstring>

#include <sys/stat.h>
#include <sys/types.h>

#include "camera.h"

typedef Matrix<Vec4f> Matrix4;

void InverseMatrix(float *matrix) {
	float out[4][4];

	float t[12], m[16];
	memcpy(m, matrix, 16 * 4);

	t[0]  = m[10] * m[15];
	t[1]  = m[11] * m[14];
	t[2]  = m[9 ] * m[15];
	t[3]  = m[11] * m[13];
	t[4]  = m[9 ] * m[14];
	t[5]  = m[10] * m[13];
	t[6]  = m[8 ] * m[15];
	t[7]  = m[11] * m[12];
	t[8]  = m[8 ] * m[14];
	t[9]  = m[10] * m[12];
	t[10] = m[8 ] * m[13];
	t[11] = m[9 ] * m[12];

	out[0][0]  = t[0] * m[5] + t[3] * m[6] + t[4 ] * m[7];
	out[0][0] -= t[1] * m[5] + t[2] * m[6] + t[5 ] * m[7];
	out[0][1]  = t[1] * m[4] + t[6] * m[6] + t[9 ] * m[7];
	out[0][1] -= t[0] * m[4] + t[7] * m[6] + t[8 ] * m[7];
	out[0][2]  = t[2] * m[4] + t[7] * m[5] + t[10] * m[7];
	out[0][2] -= t[3] * m[4] + t[6] * m[5] + t[11] * m[7];
	out[0][3]  = t[5] * m[4] + t[8] * m[5] + t[11] * m[6];
	out[0][3] -= t[4] * m[4] + t[9] * m[5] + t[10] * m[6];
	out[1][0]  = t[1] * m[1] + t[2] * m[2] + t[5 ] * m[3];
	out[1][0] -= t[0] * m[1] + t[3] * m[2] + t[4 ] * m[3];
	out[1][1]  = t[0] * m[0] + t[7] * m[2] + t[8 ] * m[3];
	out[1][1] -= t[1] * m[0] + t[6] * m[2] + t[9 ] * m[3];
	out[1][2]  = t[3] * m[0] + t[6] * m[1] + t[11] * m[3];
	out[1][2] -= t[2] * m[0] + t[7] * m[1] + t[10] * m[3];
	out[1][3]  = t[4] * m[0] + t[9] * m[1] + t[10] * m[2];
	out[1][3] -= t[5] * m[0] + t[8] * m[1] + t[11] * m[2];

	t[0]  = m[2] * m[7];
	t[1]  = m[3] * m[6];
	t[2]  = m[1] * m[7];
	t[3]  = m[3] * m[5];
	t[4]  = m[1] * m[6];
	t[5]  = m[2] * m[5];
	t[6]  = m[0] * m[7];
	t[7]  = m[3] * m[4];
	t[8]  = m[0] * m[6];
	t[9]  = m[2] * m[4];
	t[10] = m[0] * m[5];
	t[11] = m[1] * m[4];

	out[2][0]  = t[0 ] * m[13] + t[3 ] * m[14] + t[ 4] * m[15];
	out[2][0] -= t[1 ] * m[13] + t[2 ] * m[14] + t[ 5] * m[15];
	out[2][1]  = t[1 ] * m[12] + t[6 ] * m[14] + t[ 9] * m[15];
	out[2][1] -= t[0 ] * m[12] + t[7 ] * m[14] + t[ 8] * m[15];
	out[2][2]  = t[2 ] * m[12] + t[7 ] * m[13] + t[10] * m[15];
	out[2][2] -= t[3 ] * m[12] + t[6 ] * m[13] + t[11] * m[15];
	out[2][3]  = t[5 ] * m[12] + t[8 ] * m[13] + t[11] * m[14];
	out[2][3] -= t[4 ] * m[12] + t[9 ] * m[13] + t[10] * m[14];
	out[3][0]  = t[2 ] * m[10] + t[5 ] * m[11] + t[ 1] * m[ 9];
	out[3][0] -= t[4 ] * m[11] + t[0 ] * m[ 9] + t[ 3] * m[10];
	out[3][1]  = t[8 ] * m[11] + t[0 ] * m[ 8] + t[ 7] * m[10];
	out[3][1] -= t[6 ] * m[10] + t[9 ] * m[11] + t[ 1] * m[ 8];
	out[3][2]  = t[6 ] * m[9 ] + t[11] * m[11] + t[ 3] * m[ 8];
	out[3][2] -= t[10] * m[11] + t[2 ] * m[ 8] + t[ 7] * m[ 9];
	out[3][3]  = t[10] * m[10] + t[4 ] * m[ 8] + t[ 9] * m[ 9];
	out[3][3] -= t[8 ] * m[9 ] + t[11] * m[10] + t[ 5] * m[ 8];

	float iDet = 1.0f / (m[0] * out[0][0] + m[1] * out[0][1] + m[2] * out[0][2] + m[3] * out[0][3]);
	for(size_t i = 0; i < 4; i++)
		for(size_t j = 0; j < 4; j++)
			out[i][j] *= iDet;

	memcpy(matrix, out, 16 * 4);
}



void Free3dTexture(int &handle) {
	if(handle) {
		GLuint thandle = handle;
		glDeleteTextures(1, &thandle);
		handle = 0;
	}
}

namespace
{
	int shaderProgram = 0;
	int volumeHandle = 0;
	int volumeSize[3] = {0, 0, 0};

	void InitShaders() {
		if(shaderProgram) {
			GLuint handle = shaderProgram;
			glDeleteProgram(handle);
			shaderProgram = 0;
		}

		unsigned v = glCreateShader(GL_VERTEX_SHADER);
		unsigned f = glCreateShader(GL_FRAGMENT_SHADER);

		vector<char> fs;
		Loader ldr("shader.fsh");
		fs.resize(ldr.Size() + 1);
		ldr.Data(&fs[0], ldr.Size());
		fs.back() = 0;
		
		const char *fsp = &fs[0];
		const char *vs =
			"#version 130\n"
			"out vec3 texCoord;\nuniform vec3 volumeSize;\n"
			"void main() { texCoord = gl_MultiTexCoord0.xyz;\n"
			"	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
			"	gl_ClipDistance[0] = gl_Vertex.x;\n"
			"	gl_ClipDistance[1] = gl_Vertex.y;\n"
			"	gl_ClipDistance[2] = gl_Vertex.z;\n"
			"	gl_ClipDistance[3] = volumeSize.x - gl_Vertex.x;\n"
			"	gl_ClipDistance[4] = volumeSize.y - gl_Vertex.y;\n"
			"	gl_ClipDistance[5] = volumeSize.z - gl_Vertex.z;\n"
			"}\n";

		glShaderSource(v, 1, &vs, 0);
		glShaderSource(f, 1, &fsp, 0);

		glCompileShader(v);
		glCompileShader(f);

		shaderProgram = glCreateProgram();

		glAttachShader(shaderProgram, f);
		glAttachShader(shaderProgram, v);

		glLinkProgram(shaderProgram);

		{
			char buf[4096];
			glGetProgramInfoLog(shaderProgram, sizeof(buf), 0, buf);
			printf("%s\n", buf);
		}
		Assert(glGetError() == GL_NO_ERROR);
	}

	void InitMatrices(const Camera &cam, float fov, float aspect) {
		glDisable(GL_TEXTURE_2D);

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		gluPerspective(fov, aspect, 1.0f, 10000.0f);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();
		Vec3f eye = cam.pos, center = cam.pos + cam.front * 10.0f, up = cam.up;
		gluLookAt(eye.x, eye.y, eye.z, center.x, center.y, center.z, up.x, up.y, up.z);
	}

	void QuadVertex(const Vec3f &pos, const Vec3f &vScale) {
		glTexCoord3f(pos.x / vScale.x, pos.y / vScale.y, pos.z / vScale.z);
		glVertex3f(pos.x, pos.y, pos.z);
	}
	
	bool ShadersModified() {
		static time_t lastTime;
		static bool set0 = 0;
		if(!set0) {
			struct stat attrib;
			stat("dicom_viewer", &attrib);
			lastTime = attrib.st_mtime;
			set0 = 1;
		}

		struct stat attrib;
		stat("shader.fsh", &attrib);
		if(difftime(attrib.st_mtime, lastTime) > 0) {
			lastTime = attrib.st_mtime;
			return true;
		}

		return false;
	}

}

void Load3dTexture(const VolumeData &data) {
	if(volumeHandle) {
		GLuint handle = volumeHandle;
		glDeleteTextures(1, &handle);
		volumeHandle = 0;
	}

	GLuint handle;
	glGenTextures(1, &handle);
	glBindTexture(GL_TEXTURE_3D, handle);
	InputAssert(glGetError() == GL_NO_ERROR);

	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP);

	int tw = 512, th = 512, td = 512;
	int w = data.width, h = data.height, d = data.depth;
	while(tw > w * 2) tw /= 2;
	while(th > h * 2) th /= 2;
	while(td > d * 2) td /= 2;
	
	vector<u16> tdata(tw * th * td, 0);
	volumeSize[0] = tw;
	volumeSize[1] = th;
	volumeSize[2] = td;

	for(int z = 0; z < Min(td, d); z++)
		for(int y = 0; y < Min(th, h); y++)
			for(int x = 0; x < Min(tw, w); x++)
				tdata[x + (z * th + y) * tw] = data.data[x + (z * h + y) * w];

	glTexImage3D(GL_TEXTURE_3D, 0, GL_LUMINANCE_ALPHA, tw, th, td, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, &tdata[0]);

	InputAssert(glGetError() == GL_NO_ERROR);
	volumeHandle = handle;
}
void RenderVolume(const Camera &cam, float aspectRatio, int res) {
	if(!shaderProgram || ShadersModified())
		InitShaders();
	
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	InitMatrices(cam, 75.0f, aspectRatio);

	Vec3f size(volumeSize[0], volumeSize[1], volumeSize[2]);

	double planes[6][4] = {
		{ 1, 0, 0, 0 },
		{ 0, 1, 0, 0 },
		{ 0, 0, 1, 0 },
		{ -1, 0, 0, size.x },
		{ 0, -1, 0, size.y },
		{ 0, 0, -1, size.y }, };

	for(int n = 0; n < 6; n++) {
		glEnable(GL_CLIP_PLANE0 + n);
		glClipPlane(GL_CLIP_PLANE0 + n, planes[n]);
	}

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

	glActiveTexture(GL_TEXTURE0);
	glEnable(GL_TEXTURE_3D);
	glBindTexture(GL_TEXTURE_3D, volumeHandle);

	glUseProgram(shaderProgram);

	glUniform3f(glGetUniformLocation(shaderProgram, "volumeSize"), size.x, size.y, size.z);
	glUniform1i(glGetUniformLocation(shaderProgram, "volume"), 0);

	glPushMatrix();
	glLoadIdentity();

	glPopMatrix();
	float scale = Length(size) * 2;

	static Camera tcam = cam;
	
	Vec3f rayOrigin = size * 0.5f, rayDir = cam.front;
	float rayMin = 1.0f / 0.0f, rayMax = -1.0f / 0.0f;

	for(int n = 0; n < 6; n++) {
		Vec3f planeVec(planes[n][0], planes[n][1], planes[n][2]);
		float t = -((planeVec | rayOrigin) + planes[n][3]) / (planeVec | rayDir);
		bool sign = (planeVec | rayDir) < 0.0f;

		if(sign)
			rayMin = Min(rayMin, t);
		else
			rayMax = Max(rayMax, t);
	}
	rayMin = -Length(size) * 0.5f;
	rayMax = Length(size) * 0.5f;

	for(int n = res - 1; n >= 0; n--) {
		float t = n / float(res - 1);

		Vec3f center = rayOrigin + rayDir * (rayMin +  (rayMax - rayMin) * t);
		glBegin(GL_QUADS);
		glColor4f(n / float(res), 1, 1, 0.01f);
		QuadVertex(center - cam.right * scale - cam.up * scale, size);
		QuadVertex(center - cam.right * scale + cam.up * scale, size);
		QuadVertex(center + cam.right * scale + cam.up * scale, size);
		QuadVertex(center + cam.right * scale - cam.up * scale, size);
		glEnd();
	}

	for(int n = 0; n < 6; n++)
		glDisable(GL_CLIP_PLANE0 + n);

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glDisable(GL_TEXTURE_3D);
	glUseProgram(0);
}
