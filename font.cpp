#include <GL/gl.h>
#include "font.h"

	Font::Font() :font("data/fonts/font1.fnt") {
		Loader("data/fonts/font1_00.dds")&tex;
	}
	
	void Font::SetPos(const Vec2f &pos) {
		font.SetPos(Vec2f(pos.x,pos.y-height));
	}

	void Font::SetSize(const Vec2f &size) {
		font.SetSize(size);
	}

	void Font::BeginDrawing(int resx,int resy) {
		height=resy;

		glPushAttrib(GL_ALL_ATTRIB_BITS);
	
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glOrtho(0,resx,0,resy,0.0f,10.0f);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();
		glScalef(1.0f,-1.0f,1.0f);
		
		glDisable(GL_DEPTH_TEST);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D,tex.Id());

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	}

	void Font::FinishDrawing() {
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);

		glPopAttrib();
	}

	void Font::Print(const string &text) {
		Vec2f uv[1024],pos[1024];
		int count=font.GenQuads(text.c_str(),pos,uv,1024);

		glBegin(GL_QUADS);
		for(int mode=0;mode<2;mode++) {
			if(mode==0) glColor3f(0.0f,0.0f,0.0f);
			else glColor3f(1.0f,1.0f,1.0f);

			for(int n=0;n<count;n++) {
				const Vec2f *t=uv+n*4,*p=pos+n*4;
	
				for(int k=0;k<4;k++) {
					glTexCoord2f(t[k].x,t[k].y);
					glVertex3f(p[k].x+(mode?1.0f:0.0f),p[k].y+(mode?1.0f:0.0f),0.0f);
				}
			}
		}
		glEnd();
	}

