#ifndef RTRACER_ALIGNED_ALLOCATOR_H
#define RTRACER_ALIGNED_ALLOCATOR_H

#include <limits>
#include <veclib.h>


	template <class T,int alignment=64>
	class AlignedAllocator
	{
	public:
		typedef size_t size_type;
		typedef ptrdiff_t difference_type;
		typedef T* pointer;
		typedef const T* const_pointer;
		typedef T& reference;
		typedef const T& const_reference;
		typedef T value_type;

		template <class U> struct rebind { typedef AlignedAllocator<T,alignment> other; };

		template <class U> inline bool operator==(const AlignedAllocator &other) { return true; }
		template <class U> inline bool operator!=(const AlignedAllocator &other) { return false; }

		inline pointer address(reference r) const				{ return &r; }
		inline const_pointer address(const_reference c)			{ return &c; }
		inline size_type max_size() const						{ return std::numeric_limits<size_t>::max() / sizeof(T); }

		inline pointer allocate(size_type n,const void *t=0)	{ return (pointer)_mm_malloc(sizeof(T)*n,alignment); }
		inline void deallocate(pointer p,size_type n) 			{ _mm_free(p); }
		inline void construct(pointer p,const_reference c)		{ new( reinterpret_cast<void*>(p) ) T(c); }
		inline void destroy(pointer p)							{ (p)->~T(); }
	};


#endif

