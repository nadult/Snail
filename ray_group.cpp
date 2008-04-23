#include "ray_group.h"

#define MASK4(e0,e1,e2,e3)	_mm_setr_ps(UValue(e0).f,UValue(e1).f,UValue(e2).f,UValue(e3).f)


f32x4b GetSSEMaskFromBits_array[16] = {
	MASK4( 0, 0, 0, 0),
	MASK4(~0, 0, 0, 0),
	MASK4( 0,~0, 0, 0),
	MASK4(~0,~0, 0, 0),
	MASK4( 0, 0,~0, 0),
	MASK4(~0, 0,~0, 0),
	MASK4( 0,~0,~0, 0),
	MASK4(~0,~0,~0, 0),
	MASK4( 0, 0, 0,~0),
	MASK4(~0, 0, 0,~0),
	MASK4( 0,~0, 0,~0),
	MASK4(~0,~0, 0,~0),
	MASK4( 0, 0,~0,~0),
	MASK4(~0, 0,~0,~0),
	MASK4( 0,~0,~0,~0),
	MASK4(~0,~0,~0,~0),
};

#undef MASK4
