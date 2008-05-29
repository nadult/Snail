#ifndef RTRACER_GL_WINDOW_H
#define RTRACER_GL_WINDOW_H

#include "rtbase.h"
#include "image.h"


	enum KeyId {
		Key_unknown		=-1,
		Key_space		=' ',
		Key_special		=256,
		Key_esc			=(Key_special+ 1),
		Key_f1 			=(Key_special+ 2),
		Key_f2 			=(Key_special+ 3),
		Key_f3 			=(Key_special+ 4),
		Key_f4 			=(Key_special+ 5),
		Key_f5 			=(Key_special+ 6),
		Key_f6 			=(Key_special+ 7),
		Key_f7 			=(Key_special+ 8),
		Key_f8 			=(Key_special+ 9),
		Key_f9 			=(Key_special+10),
		Key_f10			=(Key_special+11),
		Key_f11			=(Key_special+12),
		Key_f12			=(Key_special+13),
		Key_up			=(Key_special+14),
		Key_down		=(Key_special+15),
		Key_left		=(Key_special+16),
		Key_right		=(Key_special+17),
		Key_lshift		=(Key_special+18),
		Key_rshift		=(Key_special+19),
		Key_lctrl		=(Key_special+20),
		Key_rctrl		=(Key_special+21),
		Key_lalt		=(Key_special+22),
		Key_ralt		=(Key_special+23),
		Key_tab			=(Key_special+24),
		Key_enter		=(Key_special+25),
		Key_backspace	=(Key_special+26),
		Key_insert		=(Key_special+27),
		Key_del			=(Key_special+28),
		Key_pageup		=(Key_special+29),
		Key_pagedown	=(Key_special+30),
		Key_home		=(Key_special+31),
		Key_end			=(Key_special+32),
		Key_kp_0		=(Key_special+33),
		Key_kp_1		=(Key_special+34),
		Key_kp_2		=(Key_special+35),
		Key_kp_3		=(Key_special+36),
		Key_kp_4		=(Key_special+37),
		Key_kp_5		=(Key_special+38),
		Key_kp_6		=(Key_special+39),
		Key_kp_7		=(Key_special+40),
		Key_kp_8		=(Key_special+41),
		Key_kp_9		=(Key_special+42),
		Key_kp_divide	=(Key_special+43),
		Key_kp_multiply	=(Key_special+44),
		Key_kp_subtract	=(Key_special+45),
		Key_kp_add		=(Key_special+46),
		Key_kp_decimal	=(Key_special+47),
		Key_kp_equal	=(Key_special+48),
		Key_kp_enter	=(Key_special+49),

		Key_last		=Key_kp_enter
	};

	class GLWindow
	{
	public:
		GLWindow(uint w,uint h,bool fullscreen);
		~GLWindow();

		void RenderImage(const Image&);
		
		bool PollEvents();
		void SwapBuffers();

		void SetSize(uint w,uint h);
		uint GetWidth() const;
		uint GetHeight() const;
		void SetTitle(const char *title);
		void GrabMouse(bool grab);

		char CharDown() const;

		bool Key(uint) const;
		bool KeyDown(uint) const;
		bool KeyUp(uint) const;

		bool MouseKey(uint) const;
		bool MouseKeyDown(uint) const;
		bool MouseKeyUp(uint) const;

		Vec3f MousePos() const;
		Vec3f MouseMove() const;

		void ShowCursor(bool);
	};

#endif

