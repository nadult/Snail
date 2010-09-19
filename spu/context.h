#ifndef RTRACER_SPU_CONTEXT_H
#define RTRACER_SPU_CONTEXT_H

#include <libspe2.h>
#include "rtbase.h"

extern spe_program_handle_t spe_trace;

struct SPEGangContext {
	SPEGangContext();
	~SPEGangContext();

	void Create();
	void Destroy();

	spe_gang_context_ptr_t ptr;
};	

struct SPEContext {
	SPEContext();
	~SPEContext();

	void Create(SPEGangContext* = 0);
	void Destroy();

	void Load(spe_program_handle_t *program);
	void Run(void *argp = 0, void *envp = 0);

	spe_context_ptr_t ptr;
};

#endif
