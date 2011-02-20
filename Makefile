all: client node

BUILD_DIR=/tmp/rtbuild

_dummy := $(shell [ -d $(BUILD_DIR) ] || mkdir -p $(BUILD_DIR))
_dummy := $(shell [ -d $(BUILD_DIR)/bvh ] || mkdir -p $(BUILD_DIR)/bvh)
_dummy := $(shell [ -d $(BUILD_DIR)/cell ] || mkdir -p $(BUILD_DIR)/cell)
_dummy := $(shell [ -d $(BUILD_DIR)/dbvh ] || mkdir -p $(BUILD_DIR)/dbvh)
_dummy := $(shell [ -d $(BUILD_DIR)/formats ] || mkdir -p $(BUILD_DIR)/formats)
_dummy := $(shell [ -d $(BUILD_DIR)/sampling ] || mkdir -p $(BUILD_DIR)/sampling)
_dummy := $(shell [ -d $(BUILD_DIR)/shading ] || mkdir -p $(BUILD_DIR)/shading)

FILES=funcs render tree_stats bounding_box gl_window rtbase base_scene bvh/traverse thread_pool \
	  camera client server node triangle font ray_generator frame_counter ray_group \
	  tex_handle bvh/tree sampling/point_sampler_dxt sampling/sampler sampling/sat_sampler \
	shading/material sampling/bilinear_sampler sampling/point_sampler16bit shading/uber_material rtracer \
	sampling/point_sampler formats/loader formats/wavefront_obj formats/doom3_proc compression \
	comm_data comm_mpi comm_tcp dbvh/tree dbvh/traverse scene scene_trace render_opengl photons

RFILES=render_opengl render bvh/traverse dbvh/traverse ray_generator scene_trace
TFILES=client server node gl_window tex_handle font rtracer comm_mpi comm_tcp comm_data
SHARED_FILES=$(filter-out $(RFILES), $(filter-out $(TFILES), $(FILES)))

LIBS=-lgfxlib -lbaselib -lpng -lz -lgomp

MPILIBS=`$(HOME)/bin/mpicxx -showme:link`

LINUX_LIBS=$(LIBS)
INCLUDES=-I$(HOME)/include/ -I./

NICE_FLAGS=-Woverloaded-virtual -Wnon-virtual-dtor
#FLAGS=-ggdb -std=gnu++0x
FLAGS=-march=native --param inline-unit-growth=1000 -std=gnu++0x -O3 -ggdb -rdynamic \
	  -ffast-math -DNDEBUG -mfpmath=sse -msse2 $(NICE_FLAGS) -pthread -fopenmp

CXX=/usr/local/gcc-4.5/bin/g++

DEPS:=$(FILES:%=$(BUILD_DIR)/%.dep)
SOURCES:=$(FILES:%=%.cpp)
OBJECTS:=$(FILES:%=$(BUILD_DIR)/%.o)
SHARED_OBJECTS:=$(SHARED_FILES:%=$(BUILD_DIR)/%.o)
RENDER_OBJECTS:=$(RFILES:%=$(BUILD_DIR)/%.o)

$(DEPS): $(BUILD_DIR)/%.dep: %.cpp
	$(CXX) $(FLAGS) $(INCLUDES) -MM $< -MT $(BUILD_DIR)/$*.o > $@

pch.h.gch: pch.h
	$(CXX) $(FLAGS) $(INCLUDES) pch.h -o $@

$(OBJECTS): $(BUILD_DIR)/%.o: pch.h.gch %.cpp
	$(CXX) $(FLAGS) $(INCLUDES) -c $*.cpp -o $@

node: $(SHARED_OBJECTS) $(RENDER_OBJECTS) $(BUILD_DIR)/server.o $(BUILD_DIR)/node.o \
		$(BUILD_DIR)/comm_mpi.o $(BUILD_DIR)/comm_tcp.o $(BUILD_DIR)/comm_data.o
	$(CXX) $(FLAGS) $(INCLUDES) -o $@ $^ $(MPILIBS) $(LINUX_LIBS) -lboost_system -lboost_regex

client: $(SHARED_OBJECTS) $(BUILD_DIR)/client.o $(BUILD_DIR)/gl_window.o \
	$(BUILD_DIR)/tex_handle.o $(BUILD_DIR)/font.o $(BUILD_DIR)/comm_tcp.o $(BUILD_DIR)/comm_data.o
	$(CXX) $(FLAGS) $(INCLUDES) -o $@ $^ -lglfw -lXrandr -lGL -lGLU \
		$(LINUX_LIBS) -lboost_system -lboost_regexs -g

rtracer: $(SHARED_OBJECTS) $(RENDER_OBJECTS) $(BUILD_DIR)/rtracer.o $(BUILD_DIR)/gl_window.o \
	$(BUILD_DIR)/tex_handle.o $(BUILD_DIR)/font.o $(BUILD_DIR)/render_opengl.o
	$(CXX) $(FLAGS) $(INCLUDES) -o $@ $^ -lglfw -lXrandr -lGL -lGLU \
		$(LINUX_LIBS) -g

ssh_all:
	ssh blader 'cd /workspace1/rtracer && make -j6 node node_ppc'

ssh_clean:
	ssh blader 'cd /workspace1/rtracer && make -j6 clean node_ppc_clean'

node_ppc: render_spu.cpp $(SOURCES) spu/*.cpp
	ssh b1 'cd /workspace1/rtracer && make -f Makefile.cell -j4 node_ppc'
	cp node_ppc /home/guests/ibmeng/rtbin/

node_ppc_clean:
	ssh b1 'cd /workspace1/rtracer && make -f Makefile.cell -j4 clean'

clean:
	-rm -f $(OBJECTS) $(DEPS) $(BUILD_DIR)/.depend pch.h.gch client node
	-rmdir $(BUILD_DIR)/formats $(BUILD_DIR)/sampling $(BUILD_DIR)/shading $(BUILD_DIR)/bvh $(BUILD_DIR)/dbvh \
		$(BUILD_DIR)/cell
	-rmdir $(BUILD_DIR)
	cd cell && make clean

$(BUILD_DIR)/.depend: $(DEPS)
	cat $(DEPS) > $(BUILD_DIR)/.depend

depend: $(BUILD_DIR)/.depend

.PHONY: clean depend ssh_all ssh_clean node_ppc_clean


DEPEND_FILE=$(BUILD_DIR)/.depend
DEP=$(wildcard $(DEPEND_FILE))
ifneq "$(DEP)" ""
include $(DEP)
endif

