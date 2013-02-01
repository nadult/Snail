#include "spu/context.h"
#include <errno.h>
#include <string.h>

SPEGangContext::SPEGangContext() :ptr(0) { }

SPEGangContext::~SPEGangContext() {
	try { Destroy(); } catch(...) { }
}

void SPEGangContext::Create() {
	assert(!ptr);

	if(!( ptr = spe_gang_context_create(0) ))
		THROW("Error while creating gang context: ", strerror(errno));
}

void SPEGangContext::Destroy() {
	if(ptr)
		if(( spe_gang_context_destroy(ptr) ) != 0)
			THROW("Error while destroying gang context: ", strerror(errno));
}

SPEContext::SPEContext() :ptr(0) { }

SPEContext::~SPEContext() {
	try { Destroy(); } catch(...) { }
}

void SPEContext::Create(SPEGangContext *gang) {
	assert(!ptr);
	if(gang) assert(gang->ptr);

	if(!( ptr = spe_context_create(0, gang?gang->ptr : 0) ))
		THROW("Error while creating context: ", strerror(errno));
}

void SPEContext::Destroy() {
	if(ptr)
		if(( spe_context_destroy(ptr) ) != 0)
			THROW("Error while destroying context: ", strerror(errno));
}

void SPEContext::Load(spe_program_handle_t *program) {
	if(spe_program_load(ptr, program) != 0)
		THROW("Error while loading program: ", strerror(errno));
}

void SPEContext::Run(void *argp, void *envp) {
	assert(ptr);

	unsigned int entry = SPE_DEFAULT_ENTRY;
	if(( spe_context_run(ptr, &entry, 0, argp, envp, 0) ) != 0)
		THROW("Error while running program: ", strerror(errno));
}
