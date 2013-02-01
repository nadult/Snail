#include "rtbase.h"
#include "tex_handle.h"
#include <gfxlib_font.h>

class Font {
public:
	Font();
	
	void SetPos(const Vec2f &pos);
	void SetSize(const Vec2f &size);
	void BeginDrawing(int resx,int resy);
	void FinishDrawing();

	void Print(const string &text);
	
	inline void PrintAt(const Vec2f &pos, const string &text) {
		SetPos(pos);
		Print(text);
	}

	gfxlib::Font font;
	TexHandle tex;
	float height;
};

