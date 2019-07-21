#pragma once

#include <tr1/type_traits>
#include "rtbase.h"

namespace comm {

	class Socket {
	public:
		Socket();
		~Socket();
		void Accept(int port);
		void Connect(const char *host, const char *port);
		void Write(const void*, size_t);
		void Read(void*, size_t);
		void NoDelay(bool);

	private:
		Socket(const Socket&);
		void operator=(const Socket&);

		void *sock, *serv;
		friend class PSocket;
	};

	struct PSocket {
		PSocket(Socket&);
		PSocket();

		void Write(const void*, size_t);
		void Read(void*, size_t);

	private:
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

	class MPIAnyNode {
	public:
		MPIAnyNode(int *rank = 0, int tag = 0) :rank(rank), tag(tag) { }
		void Read(void*, size_t);

	protected:
		int *rank, tag;
	};

	// Make sure youre writing in rank0, and reading in other ranks
	class MPIBcast {
	public:
		void Read(void*, size_t);
		void Write(const void*, size_t);
	};

	inline Socket &operator>>(Socket &sock, const Data data) {
		sock.Read(data.data, data.size);
		return sock;
	}
	
	inline Socket &operator<<(Socket &sock, const Data data) {
		sock.Write(data.data, data.size);
		return sock;
	}

	template <class T>
	inline const T operator>>(T sock, const Data data) {
		static_assert(
				std::tr1::is_same<T, MPINode>::value ||
				std::tr1::is_same<T, MPIAnyNode>::value ||
				std::tr1::is_same<T, PSocket>::value ||
				std::tr1::is_same<T, MPIBcast>::value, "burp");
		sock.Read(data.data, data.size);
		return sock;
	}
	
	template <class T>
	inline const T operator<<(T sock, const Data data) {
		static_assert(
				std::tr1::is_same<T, MPINode>::value ||
				std::tr1::is_same<T, PSocket>::value ||
				std::tr1::is_same<T, MPIBcast>::value, "burp");
		sock.Write(data.data, data.size);
		return sock;
	}

	template <class T> inline T operator<<(T node, float i)
		{ return node << Data(&i, sizeof(i)); }
	template <class T> inline T operator>>(T node, float &i)
		{ return node >> Data(&i, sizeof(i)); }

	template <class T> inline T operator<<(T node, double i)
		{ return node << Data(&i, sizeof(i)); }
	template <class T> inline T operator>>(T node, double &i)
		{ return node >> Data(&i, sizeof(i)); }
	
	template <class T> inline T operator<<(T node, bool i)
		{ return node << Data(&i, sizeof(i)); }
	template <class T> inline T operator>>(T node, bool &i)
		{ return node >> Data(&i, sizeof(i)); }

	template <class T> inline T operator<<(T node, int i)
		{ return node << Data(&i, sizeof(i)); }
	template <class T> inline T operator>>(T node, int &i)
		{ return node >> Data(&i, sizeof(i)); }
	
	template <class T> inline T operator<<(T node, unsigned int i)
		{ return node << Data(&i, sizeof(i)); }
	template <class T> inline T operator>>(T node, unsigned int &i)
		{ return node >> Data(&i, sizeof(i)); }


	template <class T> inline T operator<<(T node, Vec3f i)
		{ return node << Data(&i, sizeof(i)); }
	template <class T> inline T operator>>(T node, Vec3f &i)
		{ return node >> Data(&i, sizeof(i)); }

}

#include "camera.h"
#include "light.h"

namespace comm {

	//Dont forget to update operators << >> after modifying
	//this structure
	struct LoadNewModel {
		string name;
		int resx, resy;
		int nNodes;
		int rebuild; // 2 :rebuild slow
		bool flipNormals;
		bool swapYZ;
	};

	PSocket operator<<(PSocket sock, const LoadNewModel&);
	PSocket operator>>(PSocket sock, LoadNewModel&);

}
