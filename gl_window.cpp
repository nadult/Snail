#include <memory.h>
#include <GL/glfw.h>
#include "gl_window.h"


extern "C"
{
	extern struct {
		int  MousePosX, MousePosY;
		int  WheelPos;
		char MouseButton[ GLFW_MOUSE_BUTTON_LAST+1 ];
		char Key[ GLFW_KEY_LAST+1 ];
		int  LastChar;
		int  StickyKeys;
		int  StickyMouseButtons;
		int  KeyRepeat;
		int  MouseMoved, OldMouseX, OldMouseY;
	} _glfwInput;
}

namespace
{
	struct InputState {
		int  MousePosX, MousePosY;
		int  WheelPos;
		char MouseButton[ GLFW_MOUSE_BUTTON_LAST+1 ];
		char Key[ GLFW_KEY_LAST+1 ];
		int  LastChar;
		int  StickyKeys;
		int  StickyMouseButtons;
		int  KeyRepeat;
		int  MouseMoved, OldMouseX, OldMouseY;
	} lastInput,activeInput;

	int mouseDX,mouseDY;
	u32 key2Glfw[Key_last+1];
	bool fullScreen,initialized;

	bool wantClose,created=0;
	int GLFWCALL CloseWindowHandle() { wantClose=1; return GL_FALSE; }
}


	GLWindow::GLWindow(u32 width,u32 height,bool tFullScreen) {
		fullScreen=tFullScreen;
		if(created) {
			throw Exception("Trying to create more than one glfw window");
			return;
		}
		if(!glfwInit()) {
			throw Exception("Error while initializing GLFW");
			return;
		}
		glfwOpenWindowHint(GLFW_WINDOW_NO_RESIZE,GL_TRUE);

		if(!glfwOpenWindow(width,height,8,8,8,8,24,8,fullScreen?GLFW_FULLSCREEN:GLFW_WINDOW)) {
			throw Exception("Error while initializing window with glfwOpenGLWindow");
			return;
		}

		glfwDisable(GLFW_AUTO_POLL_EVENTS);
		glfwSwapInterval(1);
		glfwSetWindowPos(0,24);

		wantClose=0;
		glfwSetWindowCloseCallback(CloseWindowHandle);

		memset(&lastInput,0,sizeof(lastInput));
		memset(&activeInput,0,sizeof(activeInput));

		for(u32 n=0;n<256;n++) key2Glfw[n]=n;
		memset(key2Glfw+256,0,4*(Key_last+1-256));

		key2Glfw[Key_space]=GLFW_KEY_SPACE;
		key2Glfw[Key_special]=GLFW_KEY_SPECIAL;
		key2Glfw[Key_esc]=GLFW_KEY_ESC;
		key2Glfw[Key_f1]=GLFW_KEY_F1;
		key2Glfw[Key_f2]=GLFW_KEY_F2;
		key2Glfw[Key_f3]=GLFW_KEY_F3;
		key2Glfw[Key_f4]=GLFW_KEY_F4;
		key2Glfw[Key_f5]=GLFW_KEY_F5;
		key2Glfw[Key_f6]=GLFW_KEY_F6;
		key2Glfw[Key_f7]=GLFW_KEY_F7;
		key2Glfw[Key_f8]=GLFW_KEY_F8;
		key2Glfw[Key_f9]=GLFW_KEY_F9;
		key2Glfw[Key_f10]=GLFW_KEY_F10;
		key2Glfw[Key_f11]=GLFW_KEY_F11;
		key2Glfw[Key_f12]=GLFW_KEY_F12;
		key2Glfw[Key_up]=GLFW_KEY_UP;
		key2Glfw[Key_down]=GLFW_KEY_DOWN;
		key2Glfw[Key_left]=GLFW_KEY_LEFT;
		key2Glfw[Key_right]=GLFW_KEY_RIGHT;
		key2Glfw[Key_lshift]=GLFW_KEY_LSHIFT;
		key2Glfw[Key_rshift]=GLFW_KEY_RSHIFT;
		key2Glfw[Key_lctrl]=GLFW_KEY_LCTRL;
		key2Glfw[Key_rctrl]=GLFW_KEY_RCTRL;
		key2Glfw[Key_lalt]=GLFW_KEY_LALT;
		key2Glfw[Key_ralt]=GLFW_KEY_RALT;
		key2Glfw[Key_tab]=GLFW_KEY_TAB;
		key2Glfw[Key_enter]=GLFW_KEY_ENTER;
		key2Glfw[Key_backspace]=GLFW_KEY_BACKSPACE;
		key2Glfw[Key_insert]=GLFW_KEY_INSERT;
		key2Glfw[Key_del]=GLFW_KEY_DEL;
		key2Glfw[Key_pageup]=GLFW_KEY_PAGEUP;
		key2Glfw[Key_pagedown]=GLFW_KEY_PAGEDOWN;
		key2Glfw[Key_home]=GLFW_KEY_HOME;
		key2Glfw[Key_end]=GLFW_KEY_END;
		key2Glfw[Key_kp_0]=GLFW_KEY_KP_0;
		key2Glfw[Key_kp_1]=GLFW_KEY_KP_1;
		key2Glfw[Key_kp_2]=GLFW_KEY_KP_2;
		key2Glfw[Key_kp_3]=GLFW_KEY_KP_3;
		key2Glfw[Key_kp_4]=GLFW_KEY_KP_4;
		key2Glfw[Key_kp_5]=GLFW_KEY_KP_5;
		key2Glfw[Key_kp_6]=GLFW_KEY_KP_6;
		key2Glfw[Key_kp_7]=GLFW_KEY_KP_7;
		key2Glfw[Key_kp_8]=GLFW_KEY_KP_8;
		key2Glfw[Key_kp_9]=GLFW_KEY_KP_9;
		key2Glfw[Key_kp_divide]=GLFW_KEY_KP_DIVIDE;
		key2Glfw[Key_kp_multiply]=GLFW_KEY_KP_MULTIPLY;
		key2Glfw[Key_kp_subtract]=GLFW_KEY_KP_SUBTRACT;
		key2Glfw[Key_kp_add]=GLFW_KEY_KP_ADD;
		key2Glfw[Key_kp_decimal]=GLFW_KEY_KP_DECIMAL;
		key2Glfw[Key_kp_equal]=GLFW_KEY_KP_EQUAL;
		key2Glfw[Key_kp_enter]=GLFW_KEY_KP_ENTER;
		created=1;
	}

	GLWindow::~GLWindow()
	{
		glfwCloseWindow();
		glfwTerminate();
		created=0;
	}

	void GLWindow::SwapBuffers()				{ glfwSwapBuffers(); }
	void GLWindow::SetSize(uint x,uint y) 		{ glfwSetWindowSize(x,y); }

	uint GLWindow::GetWidth() const				{ int x,y; glfwGetWindowSize(&x,&y); return x; }
	uint GLWindow::GetHeight() const			{ int x,y; glfwGetWindowSize(&x,&y); return y; }
	void GLWindow::SetTitle(const char *title)	{ glfwSetWindowTitle(title); }
	void GLWindow::GrabMouse(bool grab)			{ if(grab) glfwDisable(GLFW_MOUSE_CURSOR); else glfwEnable(GLFW_MOUSE_CURSOR); }

	char GLWindow::CharDown() const {
		for(int k=0;k<255;k++) {
			if(!KeyDown(k)) continue;

			bool shift=Key(Key_lshift)||Key(Key_rshift);

			if(k>=65&&k<=90) return shift?k:k-'A'+'a';				
			if(k>=48&&k<=57&&!shift) return k;
			if(k=='0') return shift?')':'0';
			if(k=='-') return shift?'_':'-';
			if(k=='.') return shift?'>':'.';
			if(k==',') return shift?'<':',';
			if(k=='/') return shift?'?':'/';
		}
		return 0;
	}
		

	bool GLWindow::Key(uint k) const			{ return activeInput.Key[key2Glfw[k]]?1:0; }
	bool GLWindow::KeyDown(uint k) const		{ return activeInput.Key[key2Glfw[k]]&&(!lastInput.Key[key2Glfw[k]]); }
	bool GLWindow::KeyUp(uint k) const			{ return (!activeInput.Key[key2Glfw[k]])&&lastInput.Key[key2Glfw[k]]; }

	bool GLWindow::MouseKey(uint k) const		{ return activeInput.MouseButton[k]?1:0; }
	bool GLWindow::MouseKeyDown(uint k) const	{ return activeInput.MouseButton[k]&&(!lastInput.MouseButton[k]); }
	bool GLWindow::MouseKeyUp(uint k) const		{ return (!activeInput.MouseButton[k])&&lastInput.MouseButton[k]; }

	Vec3f GLWindow::MousePos() const			{ return Vec3f(f32(activeInput.MousePosX),f32(activeInput.MousePosY),f32(activeInput.WheelPos)); }
	Vec3f GLWindow::MouseMove() const			{ return Vec3f(f32(mouseDX),f32(mouseDY),f32(activeInput.WheelPos-lastInput.WheelPos)); }

	bool GLWindow::PollEvents()
	{
		for(;;) {
			glfwPollEvents();
			// Aplikacja jest zminimalizowana -> uspij na 50msec
			if(glfwGetWindowParam(GLFW_ICONIFIED)) { Sleep(0.05); continue; }
			break;
		}
		int width,height; glfwGetWindowSize(&width,&height);

		lastInput=activeInput;
		memcpy(&activeInput,&_glfwInput,sizeof(_glfwInput));

		mouseDX=activeInput.MousePosX-lastInput.MousePosX;
		mouseDY=activeInput.MousePosY-lastInput.MousePosY;
		activeInput.MousePosX=Clamp(activeInput.MousePosX,0,width-1);
		activeInput.MousePosY=Clamp(activeInput.MousePosY,0,height-1);
		glfwSetMousePos(activeInput.MousePosX,activeInput.MousePosY);

		return !wantClose;
	}

	void GLWindow::ShowCursor(bool flag) {
		if(flag) glfwEnable(GLFW_MOUSE_CURSOR);
		else glfwDisable(GLFW_MOUSE_CURSOR);
	}

	void GLWindow::RenderImage(const Image &image) {
		static GLuint texture,texW=0,texH=0;
		
		if(image.width>texW||image.height>texH) {
			if(!texW&&!texH) glGenTextures(1,&texture);
			glBindTexture(GL_TEXTURE_2D,texture);
			
			for(texW=1;texW<image.width;texW*=2);
			for(texH=1;texH<image.height;texH*=2);
			
			glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,texW,texH,0,GL_RGB,GL_UNSIGNED_BYTE,0);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glClearColor(0.0,0.0,0.0,0.0);
			glDisable(GL_DEPTH_TEST);
		}
		
		glShadeModel(GL_FLAT);
		glBindTexture(GL_TEXTURE_2D,texture);
		glEnable(GL_TEXTURE_2D);
		glTexSubImage2D(GL_TEXTURE_2D,0,0,0,image.width,image.height,GL_RGB,GL_UNSIGNED_BYTE,&image.buffer[0]);
		
		float u=float(GetWidth())/float(texW);
		float v=float(GetHeight())/float(texH);
		
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glBegin(GL_QUADS);
			glTexCoord2f(0.0, 0.0); glVertex3f(-1.0, 1.0, 0.0);
			glTexCoord2f(0.0, v  ); glVertex3f(-1.0, -1.0, 0.0);
			glTexCoord2f(u  , v  ); glVertex3f(1.0, -1.0, 0.0);
			glTexCoord2f(u  , 0.0); glVertex3f(1.0, 1.0, 0.0);
		glEnd();
	}
