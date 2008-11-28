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

	template <typename... Args>
	void Print(const Args & ...args) { Print((string)Stringize(args...)); }

	template <typename... Args>
	void PrintAt(const Vec2f &pos,const Args & ...args) { SetPos(pos); Print((string)Stringize(args...)); }

	gfxlib::Font font;
	TexHandle tex;
	float height;
};

