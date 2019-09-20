#pragma once

#include <limits>
#include <veclib.h>
#include <fwk/sys/memory.h>

	template <class T,int alignment = 64>
	class AlignedAllocator
	{
	public:
		typedef size_t size_type;
		typedef long long difference_type;
		typedef T* pointer;
		typedef const T* const_pointer;
		typedef T& reference;
		typedef const T& const_reference;
		typedef T value_type;

		template <class U> struct rebind { typedef AlignedAllocator<T,alignment> other; };

		bool operator==(const AlignedAllocator &other) { return true; }
		bool operator!=(const AlignedAllocator &other) { return false; }

		pointer address(reference r) const				{ return &r; }
		const_pointer address(const_reference c)			{ return &c; }
		size_type max_size() const						{ return std::numeric_limits<size_t>::max() / sizeof(T); }

		pointer allocate(size_type n, const void* = 0) {
			size_t size = n * sizeof(T);

			char *ptr = (char*)fwk::allocate(size + alignment + sizeof(int));

			char *ptr2 = ptr + sizeof(int);
			char *aligned_ptr = ptr2 + (alignment - ((size_t)ptr2 & (alignment - 1)));

			ptr2 = aligned_ptr - sizeof(int);
			*((int*)ptr2)=(int)(aligned_ptr - ptr);

			return (pointer)aligned_ptr;
		}

		void deallocate(pointer ptr, size_type n) {
			fwk::deallocate((char*)ptr - ((int*)ptr)[-1]);
		}
		void construct(pointer p,const_reference c)	{ new( reinterpret_cast<void*>(p) ) T(c); }
		void destroy(pointer p)						{ (p)->~T(); }
      
		template<typename... Args>
        void construct(pointer __p, Args&&... __args) { ::new((void *)__p) T(std::forward<Args>(__args)...); }
	};
