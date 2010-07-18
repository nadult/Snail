all: client node

BUILD_DIR=/tmp/build

_dummy := $(shell [ -d $(BUILD_DIR) ] || mkdir -p $(BUILD_DIR))
_dummy := $(shell [ -d $(BUILD_DIR)/bvh ] || mkdir -p $(BUILD_DIR)/bvh)
_dummy := $(shell [ -d $(BUILD_DIR)/dbvh ] || mkdir -p $(BUILD_DIR)/dbvh)
_dummy := $(shell [ -d $(BUILD_DIR)/formats ] || mkdir -p $(BUILD_DIR)/formats)
_dummy := $(shell [ -d $(BUILD_DIR)/sampling ] || mkdir -p $(BUILD_DIR)/sampling)
_dummy := $(shell [ -d $(BUILD_DIR)/shading ] || mkdir -p $(BUILD_DIR)/shading)

FILES=funcs render tree_stats bounding_box gl_window rtbase base_scene bvh/traverse thread_pool \
	  camera mesh client server_node triangle font ray_generator frame_counter ray_group \
	  tex_handle bvh/tree sampling/point_sampler_dxt sampling/sampler sampling/sat_sampler \
	shading/material sampling/bilinear_sampler sampling/point_sampler16bit obbox_fit rtracer \
	sampling/point_sampler formats/loader formats/wavefront_obj formats/doom3_proc compression \
	comm_mpi comm_tcp dbvh/tree dbvh/traverse quat

TFILES=client server_node gl_window tex_handle font rtracer comm_mpi comm_tcp
SHARED_FILES=$(filter-out $(TFILES), $(FILES))

LIBS=-lgfxlib -lbaselib -lpng -lz -lboost_system -lboost_regex

MPILIBS=`$(HOME)/bin/mpicxx -showme:link`

LINUX_LIBS=$(LIBS)
INCLUDES=-I./

NICE_FLAGS=-Woverloaded-virtual -Wnon-virtual-dtor
FLAGS=-march=native --param inline-unit-growth=1000 -rdynamic -std=gnu++0x -O3 -ggdb \
	  -ffast-math -DNDEBUG -mfpmath=sse -msse2 $(NICE_FLAGS) -fopenmp

CXX=$(GCC45)/bin/g++

DEPS:=$(FILES:%=$(BUILD_DIR)/%.dep)
SOURCES:=$(FILES:%=%.cpp)
OBJECTS:=$(FILES:%=$(BUILD_DIR)/%.o)
SHARED_OBJECTS:=$(SHARED_FILES:%=$(BUILD_DIR)/%.o)

$(DEPS): $(BUILD_DIR)/%.dep: %.cpp
	$(CXX) $(FLAGS) $(INCLUDES) -MM $< -MT $(BUILD_DIR)/$*.o > $@

pch.h.gch: pch.h
	$(CXX) $(FLAGS) $(INCLUDES) pch.h -o $@

$(OBJECTS): $(BUILD_DIR)/%.o: pch.h.gch %.cpp
	$(CXX) $(FLAGS) $(INCLUDES) -c $*.cpp -o $@

node: $(SHARED_OBJECTS) $(BUILD_DIR)/server_node.o $(BUILD_DIR)/comm_mpi.o $(BUILD_DIR)/comm_tcp.o
	$(CXX) $(FLAGS) $(INCLUDES) -o node $^ $(MPILIBS) $(LINUX_LIBS) -pthread
#	rm $(BUILD_DIR)/comm_tcp.o

client_no_link:$(SHARED_OBJECTS) $(BUILD_DIR)/gl_window.o \
	$(BUILD_DIR)/tex_handle.o $(BUILD_DIR)/font.o

client: $(SHARED_OBJECTS) $(BUILD_DIR)/client.o $(BUILD_DIR)/gl_window.o \
	$(BUILD_DIR)/tex_handle.o $(BUILD_DIR)/font.o $(BUILD_DIR)/comm_tcp.o
	$(CXX) $(FLAGS) $(INCLUDES) -o client $^ -lglfw -lXrandr -lGL -lGLU \
		$(LINUX_LIBS)  -pthread -pg -g

rtracer: $(SHARED_OBJECTS) $(BUILD_DIR)/rtracer.o $(BUILD_DIR)/gl_window.o \
	$(BUILD_DIR)/tex_handle.o $(BUILD_DIR)/font.o
	$(CXX) $(FLAGS) $(INCLUDES) -o rtracer $^ -lglfw -lXrandr -lGL -lGLU \
		$(LINUX_LIBS)  -pthread -pg -g

clean:
	-rm -f $(OBJECTS) $(DEPS) $(BUILD_DIR)/.depend pch.h.gch client node
	-rmdir $(BUILD_DIR)/formats $(BUILD_DIR)/sampling $(BUILD_DIR)/shading $(BUILD_DIR)/bvh $(BUILD_DIR)/dbvh
	-rmdir $(BUILD_DIR)

$(BUILD_DIR)/.depend: $(DEPS)
	cat $(DEPS) > $(BUILD_DIR)/.depend

depend: $(BUILD_DIR)/.depend

.PHONY: clean depend client_no_link

DEPEND_FILE=$(BUILD_DIR)/.depend
DEP=$(wildcard $(DEPEND_FILE))
ifneq "$(DEP)" ""
include $(DEP)
endif

