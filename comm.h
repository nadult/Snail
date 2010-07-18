#ifndef RTRACER_COMM_H
#define RTRACER_COMM_H

#include <tr1/type_traits>

namespace comm {

	class Socket {
	public:
		Socket();
		~Socket();
		void Accept(int port);
		void Connect(const char *host, const char *port);
		void Write(const void*, size_t);
		void Read(void*, size_t);

	private:
		Socket(const Socket&);
		void operator=(const Socket&);

		void *sock, *serv;
	};

	struct Data {
		Data(void *data, size_t size) :data(data), size(size) { }
		void *data;
		size_t size;
	};

	template <class T>
	const Data Pod(T &t) {
		return Data(&t, sizeof(T));
	}

	inline Socket &operator>>(Socket &sock, const Data data) {
		sock.Read(data.data, data.size);
		return sock;
	}
	
	inline Socket &operator<<(Socket &sock, const Data data) {
		sock.Write(data.data, data.size);
		return sock;
	}

	// Handle for writing to / reading from MPI node
	// tag is increased by 1 with every operation
	class MPINode {
	public:
		MPINode(int rank, int tag = 0) :rank(rank), tag(tag) { }

		void Write(const void*, size_t);
		void Read(void*, size_t);
		
	protected:
		int rank, tag;
	};

	// Handle for reading from any / writing to all MPI nodes
	class MPIAnyNode {
	public:
		MPIAnyNode(int *rank = 0, int tag = 0) :rank(rank), tag(tag) { }

		void Write(void*, size_t);
		void Read(void*, size_t);

	protected:
		int *rank, tag;
	};

	inline const MPIAnyNode operator>>(MPIAnyNode node, const Data data) {
		node.Read(data.data, data.size);
		return node;
	}
	
	inline const MPIAnyNode operator<<(MPIAnyNode node, const Data data) {
		node.Write(data.data, data.size);
		return node;
	}

	inline const MPINode operator>>(MPINode node, const Data data) {
		node.Read(data.data, data.size);
		return node;
	}
	
	inline const MPINode operator<<(MPINode node, const Data data) {
		node.Write(data.data, data.size);
		return node;
	}

}

#endif
