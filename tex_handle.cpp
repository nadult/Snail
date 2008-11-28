#include "tex_handle.h"
#include <squish.h>
#include "rtbase.h"
#include <GL/gl.h>

	namespace
	{
		void TestGlError(const char *msg) {
			int err=glGetError();
			if(err==GL_NO_ERROR) return;

			switch(err) {
		#define CASE(e) case e: throw Exception(Stringizer() + msg + ": " #e );
				CASE(GL_INVALID_ENUM)
				CASE(GL_INVALID_VALUE)
				CASE(GL_INVALID_OPERATION)
				CASE(GL_STACK_OVERFLOW)
				CASE(GL_STACK_UNDERFLOW)
				CASE(GL_OUT_OF_MEMORY)
				default: ThrowException(msg);
		#undef CASE
			}
		}

		typedef TexHandle::Format Format;

		void SetTextureData(u32 level,const Format &fmt,u32 width,u32 height,void *pixels) {
			if((width&(width-1))||(height&(height-1)))
				ThrowException("Texture width or height is not a power of 2");

			if(fmt.IsCompressed()) ThrowException("Texture compression is not supported");
				
			glTexImage2D(GL_TEXTURE_2D,level,fmt.GlInternal(),width,height,0,fmt.GlFormat(),fmt.GlType(),pixels);
			TestGlError("Error while loading texture surface to the device");
		}

		void GetTextureData(u32 level,const Format &fmt,void *pixels) {
			if(fmt.IsCompressed())
				TestGlError("Error while loading compressed texture surface to the device");
				
			glGetTexImage(GL_TEXTURE_2D,level,fmt.GlFormat(),fmt.GlType(),pixels);
			TestGlError("Error while loading texture surface from the device");
		}

		bool packFlagsSet=0;
	}

	TexHandle::TexHandle() :mips(0),id(~0) {
		if(!packFlagsSet) {
			glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
			glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
			glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
			glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
			glPixelStorei(GL_UNPACK_SWAP_BYTES, GL_FALSE);
			packFlagsSet=1;
		}
	}

	TexHandle::~TexHandle() {
		Free();
	}

	void TexHandle::Free() {
		if(id!=~0) {
			glDeleteTextures(1,&id);
			id=~0;
		}
	}
	
	void TexHandle::Create(uint tMips) {
		Free();
		
		glGenTextures(1,(GLuint*)&id);
		if(glGetError()!=GL_NO_ERROR)
			ThrowException("Error while creating texture");
		
		::glBindTexture(GL_TEXTURE_2D, id);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,tMips>1?GL_LINEAR_MIPMAP_LINEAR:GL_LINEAR);
		mips=tMips;
	}

	void TexHandle::SetSurface(const Surface &in) {
		Create(in.Mips());
		for(uint mip=0;mip<mips;mip++)
			SetTextureData(mip,in.GetFormat(),in.Width(mip),in.Height(mip),in.DataPointer(mip));
	}

	void TexHandle::GetSurface(Surface& out) {
		Format fmt=Format();
		out.Set(Width(),Height(),fmt,mips);
		for(uint mip=0;mip<mips;mip++)
			GetTextureData(mip,fmt,out.DataPointer(mip));
	}

	void TexHandle::Serialize(Serializer &sr) {
		Surface surface;
		if(sr.IsSaving()) GetSurface(surface);
		sr&surface;
		if(sr.IsLoading()) {
			try {
				SetSurface(surface);
			}
			catch(const Exception &ex) {
				throw Exception(Stringizer()+ex.what()+": "+sr.StreamName());
			}
		}
	}
	
	void TexHandle::CreateMip(uint level,uint w,uint h,Format fmt) {
		assert(id!=~0);

		u32 out=0,tId;
		glGetIntegerv(GL_TEXTURE_2D_BINDING_EXT,(GLint*)&tId);
		::glBindTexture(GL_TEXTURE_2D, id);
		
		glTexImage2D(GL_TEXTURE_2D,level,fmt.GlInternal(),w,h,0,fmt.GlFormat(),fmt.GlType(),0);
					
		::glBindTexture(GL_TEXTURE_2D, tId);
	}
	
	void TexHandle::UpdateMip(uint mip,uint x,uint y,uint w,uint h,void *pix,uint pixelsInRow) {
		assert(id!=~0);

		u32 out=0,tId;
		glGetIntegerv(GL_TEXTURE_2D_BINDING_EXT,(GLint*)&tId);
		::glBindTexture(GL_TEXTURE_2D, id);
	
		glPixelStorei(GL_UNPACK_ROW_LENGTH,pixelsInRow);
		glTexSubImage2D(GL_TEXTURE_2D,0,x,y,w,h,GL_RGB,GL_UNSIGNED_BYTE,pix);
		glPixelStorei(GL_UNPACK_ROW_LENGTH,0);
					
		::glBindTexture(GL_TEXTURE_2D, tId);
	}

	uint TexHandle::Width() const {
		assert(id!=~0);

		u32 out=0,tId;
		glGetIntegerv(GL_TEXTURE_2D_BINDING_EXT,(GLint*)&tId);
		::glBindTexture(GL_TEXTURE_2D, id);
		glGetTexLevelParameteriv(GL_TEXTURE_2D,0,GL_TEXTURE_WIDTH,(GLint*)&out);
		::glBindTexture(GL_TEXTURE_2D, tId);
		return out;
	}

	uint TexHandle::Height() const {
		assert(id!=~0);

		u32 out=0,tId;
		glGetIntegerv(GL_TEXTURE_2D_BINDING_EXT,(GLint*)&tId);
		::glBindTexture(GL_TEXTURE_2D, id);
		glGetTexLevelParameteriv(GL_TEXTURE_2D,0,GL_TEXTURE_HEIGHT,(GLint*)&out);
		::glBindTexture(GL_TEXTURE_2D, tId);
		return out;
	}

	Format TexHandle::GetFormat() const {
		assert(id!=~0);

		u32 internal=0,tId;
		glGetIntegerv(GL_TEXTURE_2D_BINDING_EXT,(GLint*)&tId);
		::glBindTexture(GL_TEXTURE_2D, id);
		glGetTexLevelParameteriv(GL_TEXTURE_2D,0,GL_TEXTURE_INTERNAL_FORMAT,(GLint*)&internal);
		::glBindTexture(GL_TEXTURE_2D, tId);
		return Format(internal);
	}
	
