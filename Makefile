ICCFLAGS=-openmp -inline-level=2 -inline-forceinline -fp-model fast -fp-speculationfast -shared-intel -lstdc++
GCCFLAGS=-fopenmp -mfpmath=sse -ffast-math -funroll-loops

FLAGS=$(GCCFLAGS) -O3 -msse3 -pthread -I /home/someone/rtracer/veclib
CC=g++
#CC=icc

all: rtracer

rtracer.gch: rtracer.h kdtree.h kdtraversal.h
#	$(CC) $(FLAGS) -o rtracer.gch rtracer.h
	touch rtracer.gch

asmsrc: rtracer.cpp rtracer.gch
	$(CC) $(FLAGS) -c -o src rtracer.cpp -S

rtracer.o: rtracer.cpp rtracer.gch
	$(CC) $(FLAGS) -c -o rtracer.o rtracer.cpp

image.o: image.cpp rtracer.gch
	$(CC) $(FLAGS) -c -o image.o image.cpp

kdtree.o: kdtree.cpp rtracer.gch
	$(CC) $(FLAGS) -c -o kdtree.o kdtree.cpp

sdlout.o: sdlout.cpp rtracer.gch
	$(CC) $(FLAGS) -c -o sdlout.o sdlout.cpp

ray_generator.o: ray_generator.cpp rtracer.gch
	$(CC) $(FLAGS) -c -o ray_generator.o ray_generator.cpp

model.o: model.cpp rtracer.gch
	$(CC) $(FLAGS) -c -o model.o model.cpp

rtracer: rtracer.o image.o kdtree.o sdlout.o ray_generator.o model.o rtracer.gch
	$(CC) $(FLAGS) `sdl-config --static-libs` -o rtracer rtracer.o image.o kdtree.o sdlout.o ray_generator.o model.o

clean:
	rm rtracer *.o *.gch
