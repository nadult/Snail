ICCFLAGS=-openmp -inline-level=2 -inline-forceinline -fp-model fast -fp-speculationfast -shared-intel -lstdc++
GCCFLAGS=-mfpmath=sse -ffast-math -funroll-loops -fopenmp -fno-rtti -g -DNDEBUG

FLAGS=$(GCCFLAGS) -O3 -mssse3 -pthread -I /home/someone/veclib -I /home/someone/baselib
CC=/usr/local/gcc-4.3/bin/g++
#CC=icc

all: rtracer

rtbase.gch: *.h Makefile
	$(CC) $(FLAGS) -o rtbase.gch rtbase.h

asmsrc: rtracer.cpp rtbase.gch
	$(CC) $(FLAGS) -c -o build/src rtracer.cpp -S

build/rtracer.o: rtracer.cpp rtbase.gch
	$(CC) $(FLAGS) -c -o build/rtracer.o rtracer.cpp

build/image.o: image.cpp rtbase.gch
	$(CC) $(FLAGS) -c -o build/image.o image.cpp

build/kdtree.o: kdtree.cpp rtbase.gch
	$(CC) $(FLAGS) -c -o build/kdtree.o kdtree.cpp

build/object.o: object.cpp rtbase.gch
	$(CC) $(FLAGS) -c -o build/object.o object.cpp

build/sdl_output.o: sdl_output.cpp rtbase.gch
	$(CC) $(FLAGS) -c -o build/sdl_output.o sdl_output.cpp

build/ray_generator.o: ray_generator.cpp rtbase.gch
	$(CC) $(FLAGS) -c -o build/ray_generator.o ray_generator.cpp

build/loader.o: loader.cpp rtbase.gch
	$(CC) $(FLAGS) -c -o build/loader.o loader.cpp

rtracer: build/rtracer.o build/image.o build/kdtree.o build/sdl_output.o build/ray_generator.o \
		build/loader.o build/object.o rtbase.gch
	$(CC) $(FLAGS) `sdl-config --static-libs`  -o rtracer build/rtracer.o build/image.o \
		build/kdtree.o 	build/sdl_output.o build/ray_generator.o build/loader.o build/object.o -L /home/someone/baselib -lbaselib

clean:
	rm rtracer build/*.o *.gch

# DO NOT DELETE
