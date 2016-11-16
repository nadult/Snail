all: client node rtracer

BUILD_DIR=build

_dummy := $(shell [ -d $(BUILD_DIR) ] || mkdir -p $(BUILD_DIR))
_dummy := $(shell [ -d $(BUILD_DIR)/bvh ] || mkdir -p $(BUILD_DIR)/bvh)
_dummy := $(shell [ -d $(BUILD_DIR)/cell ] || mkdir -p $(BUILD_DIR)/cell)
_dummy := $(shell [ -d $(BUILD_DIR)/dbvh ] || mkdir -p $(BUILD_DIR)/dbvh)
_dummy := $(shell [ -d $(BUILD_DIR)/formats ] || mkdir -p $(BUILD_DIR)/formats)
_dummy := $(shell [ -d $(BUILD_DIR)/sampling ] || mkdir -p $(BUILD_DIR)/sampling)
_dummy := $(shell [ -d $(BUILD_DIR)/shading ] || mkdir -p $(BUILD_DIR)/shading)

FILES=\
	funcs render tree_stats bounding_box rtbase base_scene bvh/traverse thread_pool \
	camera client server node triangle ray_generator frame_counter ray_group bvh/tree \
	shading/material  shading/uber_material rtracer mipmap_texture \
	comm_data comm_mpi comm_tcp dbvh/tree dbvh/traverse scene scene_trace render_opengl photons dicom_viewer \
	volume_data vtree vrender_opengl \
	formats/loader formats/wavefront_obj formats/doom3_proc compression \
	sampling/point_sampler sampling/sampler sampling/sat_sampler sampling/bilinear_sampler
	#sampling/point_sampler_dxt

RFILES=render_opengl render bvh/traverse dbvh/traverse ray_generator scene scene_trace photons
TFILES=client server node rtracer comm_mpi comm_tcp comm_data dicom_viewer
SHARED_FILES=$(filter-out $(RFILES), $(filter-out $(TFILES), $(FILES)))

FWK_DIR=libfwk
include $(FWK_DIR)/Makefile.include

MPILIBS=`mpicxx -showme:link`

LINUX_LIBS=$(LINUX_FWK_LIBS) $(shell $(LINUX_PKG_CONFIG) --libs $(LIBS))
MINGW_LIBS=$(MINGW_FWK_LIBS) $(shell $(MINGW_PKG_CONFIG) --libs $(LIBS))

INCLUDES=-I./ -Iveclib/ $(FWK_INCLUDES)

NICE_FLAGS=-Woverloaded-virtual -Wnon-virtual-dtor
FLAGS=-march=native -std=c++14 -O3 -ggdb -rdynamic \
	  -DNDEBUG -mfpmath=sse -msse2 $(NICE_FLAGS) -pthread -fopenmp

CXX=g++

DEPS:=$(FILES:%=$(BUILD_DIR)/%.d) $(BUILD_DIR)/pch.h.d
SOURCES:=$(FILES:%=%.cpp)
OBJECTS:=$(FILES:%=$(BUILD_DIR)/%.o)
SHARED_OBJECTS:=$(SHARED_FILES:%=$(BUILD_DIR)/%.o)
RENDER_OBJECTS:=$(RFILES:%=$(BUILD_DIR)/%.o)

PCH_FILE=$(BUILD_DIR)/pch.h.gch
$(PCH_FILE): pch.h
	cp pch.h $(BUILD_DIR)/pch.h
	$(CXX) -MMD $(FLAGS) $(INCLUDES) $(BUILD_DIR)/pch.h -o $@

$(OBJECTS): $(BUILD_DIR)/%.o: $(PCH_FILE) %.cpp
	$(CXX) -MMD $(FLAGS) $(INCLUDES) -include $(BUILD_DIR)/pch.h -c $*.cpp -o $@

node: $(SHARED_OBJECTS) $(RENDER_OBJECTS) $(BUILD_DIR)/server.o $(BUILD_DIR)/node.o \
		$(BUILD_DIR)/comm_mpi.o $(BUILD_DIR)/comm_tcp.o $(BUILD_DIR)/comm_data.o $(LINUX_FWK_LIB)
	$(CXX) $(FLAGS) $(INCLUDES) -o $@ $^ $(MPILIBS) $(LINUX_LIBS) -lboost_system -lboost_regex -lGL -lGLU

client: $(SHARED_OBJECTS) $(BUILD_DIR)/client.o $(BUILD_DIR)/comm_tcp.o $(BUILD_DIR)/comm_data.o $(LINUX_FWK_LIB)
	$(CXX) $(FLAGS) $(INCLUDES) -o $@ $^ -lglfw -lGL -lGLU \
		$(LINUX_LIBS) -lboost_system -lboost_regex -g

dicom_viewer: $(SHARED_OBJECTS) $(BUILD_DIR)/dicom_viewer.o	$(BUILD_DIR)/camera.o $(LINUX_FWK_LIB)
	$(CXX) $(FLAGS) $(INCLUDES) -o $@ $^ -lglfw -lGL -lGLU \
		$(LINUX_LIBS) -g

rtracer: $(SHARED_OBJECTS) $(RENDER_OBJECTS) $(BUILD_DIR)/rtracer.o $(BUILD_DIR)/render_opengl.o $(LINUX_FWK_LIB)
	$(CXX) $(FLAGS) $(INCLUDES) -o $@ $^ -lglfw -lGL -lGLU \
		$(LINUX_LIBS) -g

rtclean:
	-rm -f $(OBJECTS) $(DEPS) $(PCH_FILE) $(BUILD_DIR)/pch.h client node rtracer dicom_viewer
	find $(BUILD_DIR) -type d -empty -delete
	-rmdir $(BUILD_DIR)

clean: rtclean
	$(MAKE) -C $(FWK_DIR) clean

.PHONY: clean rtclean 

-include $(DEPS)
