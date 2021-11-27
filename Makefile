all: programs

FWK_DIR       = libfwk/
FWK_MODE     ?= devel
FWK_GEOM      = disabled

CFLAGS        = -Iveclib/ -Isrc/ -fopenmp
LDFLAGS_gcc   = -lgomp
LDFLAGS_clang = -fopenmp
PCH_SOURCE   := src/rtracer_pch.h
BUILD_DIR     = build/$(PLATFORM)_$(MODE)

include $(FWK_DIR)Makefile-shared

# --- Creating necessary sub-directories ----------------------------------------------------------

SUBDIRS        = build
BUILD_SUBDIRS  = bvh cell dbvh formats sampling shading

ifndef JUNK_GATHERING
_dummy := $(shell mkdir -p $(SUBDIRS))
_dummy := $(shell mkdir -p $(addprefix $(BUILD_DIR)/,$(BUILD_SUBDIRS)))
endif

# --- List of source files ------------------------------------------------------------------------


SHARED_SRC := \
	funcs tree_stats bounding_box rtbase base_scene thread_pool \
	camera triangle frame_counter ray_group bvh/tree \
	shading/material shading/uber_material mipmap_texture \
	dbvh/tree volume_data vtree vrender_opengl \
	formats/loader formats/wavefront_obj formats/doom3_proc \
	sampling/point_sampler sampling/sampler sampling/sat_sampler sampling/bilinear_sampler
	#sampling/point_sampler_dxt

RENDER_SRC     := render_opengl render bvh/traverse dbvh/traverse ray_generator scene scene_trace photons
PROGRAM_SRC    := client server node rtracer dicom_viewer

ALL_SRC        := $(PROGRAM_SRC) $(SHARED_SRC) $(RENDER_SRC)

OBJECTS        := $(ALL_SRC:%=$(BUILD_DIR)/%.o)
SHARED_OBJECTS := $(SHARED_SRC:%=$(BUILD_DIR)/%.o)
RENDER_OBJECTS := $(RENDER_SRC:%=$(BUILD_DIR)/%.o)

PROGRAMS       := $(PROGRAM_SRC:%=%$(PROGRAM_SUFFIX))
programs:         $(PROGRAMS)

# --- Build targets -------------------------------------------------------------------------------

#time -o stats.txt -a -f "%U $@" 
$(OBJECTS): $(BUILD_DIR)/%.o:  src/%.cpp $(PCH_TARGET)
	$(COMPILER) -MMD $(CFLAGS) $(PCH_CFLAGS) -c src/$*.cpp -o $@

$(PROGRAMS): %: $(SHARED_OBJECTS) $(BUILD_DIR)/%.o $(FWK_LIB_FILE)
	$(LINKER) -o $@ $^ -Wl,--export-dynamic $(LDFLAGS)

rtracer: $(RENDER_OBJECTS)

DEPS:=$(ALL_SRC:%=$(BUILD_DIR)/%.d) $(PCH_TEMP).d

JUNK_FILES    := $(OBJECTS) $(DEPS)
JUNK_DIRS     := $(SUBDIRS)

# --- Other stuff ---------------------------------------------------------------------------------

# Recreates dependency files, in case they got outdated
depends: $(PCH_TARGET)
	@echo $(ALL_SRC) | tr '\n' ' ' | xargs -P16 -t -d' ' -I '{}' $(COMPILER) $(CFLAGS) $(PCH_CFLAGS) \
		src/'{}'.cpp -MM -MF $(BUILD_DIR)/'{}'.d -MT $(BUILD_DIR)/'{}'.o -E > /dev/null

.PHONY: clean clean-libfwk clean-all print-junk

-include $(DEPS)
