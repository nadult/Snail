#include "comm.h"
#include <boost/asio.hpp>
#include <baselib.h>

using namespace baselib;
using boost::asio::ip::tcp;
using boost::asio::io_service;

namespace comm {

	Socket::Socket() {
		serv = new io_service;
		try {
			sock = new tcp::socket(*(io_service*)serv);
		}
		catch(...) {
			delete (io_service*)serv;
			throw;
		}
	}

	void Socket::Accept(int port) {
		tcp::acceptor acceptor(*(io_service*)serv, tcp::endpoint(tcp::v4(), port));
		acceptor.accept(*(tcp::socket*)sock);
		((tcp::socket*)sock)->set_option(tcp::no_delay(true));
	}

	void Socket::Connect(const char *host, const char *port) {
		tcp::resolver resolver(*(io_service*)serv);
		tcp::resolver::query query(host, port);
		tcp::resolver::iterator endpointIterator = resolver.resolve(query);
		tcp::resolver::iterator end;

		boost::system::error_code error = boost::asio::error::host_not_found;
		while(error && endpointIterator != end) {
			((tcp::socket*)sock)->close();
			((tcp::socket*)sock)->connect(*endpointIterator++, error);
		}
		if(error)
			ThrowException("Error while connecting to ", host, ':', port, " : ", error);
		
		((tcp::socket*)sock)->set_option(tcp::no_delay(true));
	}

	void Socket::Write(const void *data, size_t size) {
		boost::asio::write(*((tcp::socket*)sock), boost::asio::buffer(data, size));
	}

	void Socket::Read(void *data, size_t size) {
		int bytes = 0;
		while(bytes < size) {
			int len = ((tcp::socket*)sock)->read_some(boost::asio::buffer(((char*)data) + bytes, size - bytes));
			bytes += len;
		}
	}

	Socket::~Socket() {
		delete (tcp::socket*)sock;
		delete (io_service*)serv;
	}


}
