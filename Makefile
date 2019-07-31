MINGW_PREFIX=x86_64-w64-mingw32.static.posix-
BUILD_DIR=build
LINUX_CXX=clang++

-include Makefile.local

ifndef LINUX_LINK
LINUX_LINK=$(LINUX_CXX)
endif

ifneq (,$(findstring clang,$(LINUX_CXX)))
	CLANG=yes
	LINUX_LINK+=-fuse-ld=gold
endif

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
FLAGS=-std=c++17 -O3 -ggdb -DNDEBUG -mfpmath=sse -msse2 $(NICE_FLAGS) -pthread -fopenmp
LINUX_FLAGS=$(FLAGS) $(INCLUDES)

SOURCES:=$(FILES:%=%.cpp)
OBJECTS:=$(FILES:%=$(BUILD_DIR)/%.o)
SHARED_OBJECTS:=$(SHARED_FILES:%=$(BUILD_DIR)/%.o)
RENDER_OBJECTS:=$(RFILES:%=$(BUILD_DIR)/%.o)

# --- Precompiled headers -------------------------------------------------------------------------

PCH_FILE_SRC=rtracer_pch.h

PCH_FILE_H=$(BUILD_DIR)/pch.h
PCH_FILE_GCH=$(BUILD_DIR)/pch.h.gch
PCH_FILE_PCH=$(BUILD_DIR)/pch.h.pch

ifdef CLANG
	PCH_INCLUDE=-include-pch $(PCH_FILE_PCH)
	PCH_FILE_MAIN=$(PCH_FILE_PCH)

	ifneq ("$(wildcard libfwk/checker.so)","")
		LINUX_CHECKER_FLAGS+=-Xclang -load -Xclang  $(realpath libfwk/checker.so) -Xclang -add-plugin -Xclang check-error-attribs
	endif
else
	PCH_INCLUDE=-I$(BUILD_DIR) -include $(PCH_FILE_H)
	PCH_FILE_MAIN=$(PCH_FILE_GCH)
endif


$(PCH_FILE_H): $(PCH_FILE_SRC)
	cp $^ $@
$(PCH_FILE_MAIN): $(PCH_FILE_H)
	$(LINUX_CXX) -x c++-header -MMD $(LINUX_FLAGS) $(PCH_FILE_H) -o $@

# --- Main build commands -------------------------------------------------------------------------

DEPS:=$(FILES:%=$(BUILD_DIR)/%.d) $(PCH_FILE_H).d

$(OBJECTS): $(BUILD_DIR)/%.o: $(PCH_FILE_MAIN) %.cpp
	$(LINUX_CXX) -MMD $(LINUX_FLAGS) $(PCH_INCLUDE) -c $*.cpp -o $@

node: $(SHARED_OBJECTS) $(RENDER_OBJECTS) $(BUILD_DIR)/server.o $(BUILD_DIR)/node.o \
		$(BUILD_DIR)/comm_mpi.o $(BUILD_DIR)/comm_tcp.o $(BUILD_DIR)/comm_data.o $(LINUX_FWK_LIB)
	$(LINUX_LINK) $(LINUX_FLAGS) -o $@ $^ $(MPILIBS) $(LINUX_LIBS) -lboost_system -lboost_regex -lGL -lGLU

client: $(SHARED_OBJECTS) $(BUILD_DIR)/client.o $(BUILD_DIR)/comm_tcp.o $(BUILD_DIR)/comm_data.o $(LINUX_FWK_LIB)
	$(LINUX_LINK) $(LINUX_FLAGS) -o $@ $^ \
		$(LINUX_LIBS) -lboost_system -lboost_regex -g

dicom_viewer: $(SHARED_OBJECTS) $(BUILD_DIR)/dicom_viewer.o	$(BUILD_DIR)/camera.o $(LINUX_FWK_LIB)
	$(LINUX_LINK) $(LINUX_FLAGS) -o $@ $^ $(LINUX_LIBS) -g

rtracer: $(SHARED_OBJECTS) $(RENDER_OBJECTS) $(BUILD_DIR)/rtracer.o $(BUILD_DIR)/render_opengl.o $(LINUX_FWK_LIB)
	$(LINUX_LINK) $(LINUX_FLAGS) -o $@ $^ $(LINUX_LIBS) -g

# Recreates dependency files, in case they got outdated
depends: $(PCH_FILE_MAIN)
	@echo $(FILES) | tr '\n' ' ' | xargs -P16 -t -d' ' -I '{}' $(LINUX_CXX) $(LINUX_FLAGS) $(PCH_INCLUDE) \
		'{}'.cpp -MM -MF $(BUILD_DIR)/'{}'.d -MT $(BUILD_DIR)/'{}'.o -E > /dev/null

libfwk-clean:
	$(MAKE) -C $(FWK_DIR) clean

clean:
	-rm -f $(OBJECTS) $(DEPS) client node rtracer dicom_viewer $(PCH_FILE_GCH) $(PCH_FILE_PCH) $(PCH_FILE_H)
	find $(BUILD_DIR) -type d -empty -delete

full-clean: clean libfwk-clean

.PHONY: clean libfwk-clean full-clean

-include $(DEPS)
