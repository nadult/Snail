#ifndef RTRACER_ALIGNED_ALLOCATOR_H
#define RTRACER_ALIGNED_ALLOCATOR_H

#include <limits>
#include <veclib.h>

	template <class T,int alignment = 64>
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

		template <class U> bool operator==(const AlignedAllocator &other) { return true; }
		template <class U> bool operator!=(const AlignedAllocator &other) { return false; }

		pointer address(reference r) const				{ return &r; }
		const_pointer address(const_reference c)			{ return &c; }
		size_type max_size() const						{ return std::numeric_limits<size_t>::max() / sizeof(T); }

		pointer allocate(size_type n, const void* = 0) {
			size_t size = n * sizeof(T);

			char *ptr = (char *)malloc(size + alignment + sizeof(int));
			if(!ptr) throw std::bad_alloc();

			char *ptr2 = ptr + sizeof(int);
			char *aligned_ptr = ptr2 + (alignment - ((size_t)ptr2 & (alignment - 1)));

			ptr2 = aligned_ptr - sizeof(int);
			*((int*)ptr2)=(int)(aligned_ptr - ptr);

			return (pointer)aligned_ptr;
		}

		void deallocate(pointer ptr, size_type n) {
			free((char*)ptr - ((int*)ptr)[-1]);
		}
		void construct(pointer p,const_reference c)	{ new( reinterpret_cast<void*>(p) ) T(c); }
		void destroy(pointer p)						{ (p)->~T(); }
	};


#endif

