#include "task_switcher.h"
#include <omp.h>

int GetCoresNum()
{
#ifdef _OPENMP
	return omp_get_num_procs();
#else
	return 1;
#endif
}

