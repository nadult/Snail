#include "comm.h"
#include <mpi.h>
#include <stdlib.h>

namespace comm {

	void MPINode::Write(const void *data, size_t size) {
		MPI_Send((void*)data, size, MPI_CHAR, rank, tag++, MPI_COMM_WORLD);
	}

	void MPINode::Read(void *data, size_t size) {
		MPI_Status status;
		MPI_Recv(data, size, MPI_CHAR, rank, tag++, MPI_COMM_WORLD, &status);
	}

	void MPIAnyNode::Read(void *data, size_t size) {
		MPI_Status status;
		MPI_Recv(data, size, MPI_CHAR, MPI_ANY_SOURCE, tag, MPI_COMM_WORLD, &status);
		if(rank)
			*rank = status.MPI_SOURCE;
	}

	void MPIAnyNode::Write(void *data, size_t size) {
		//TODO: test
		exit(0);
		MPI_Bcast(data, size, MPI_CHAR, 0, MPI_COMM_WORLD);
	}

}
