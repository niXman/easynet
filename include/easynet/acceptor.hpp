
// Copyright (c) 2013,2014 niXman (i dotty nixman doggy gmail dotty com)
// All rights reserved.
//
// This file is part of EASYNET(https://github.com/niXman/easynet) project.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice, this
//   list of conditions and the following disclaimer in the documentation and/or
//   other materials provided with the distribution.
//
//   Neither the name of the {organization} nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
// ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef _easynet__acceptor_hpp
#define _easynet__acceptor_hpp

#include <easynet/socket.hpp>

namespace easynet {

/***************************************************************************/

struct acceptor: private boost::noncopyable {
	acceptor(boost::asio::io_service& ios, const std::string& ip, boost::uint16_t port)
		:acc(ios, endpoint(boost::asio::ip::address::from_string(ip), port)),
		run(true)
	{}

	boost::asio::io_service& get_io_service() { return acc.get_io_service(); }

	void cancel() { acc.cancel(); }
	void cancel(boost::system::error_code& ec) { acc.cancel(ec); }
	void close() { acc.close(); }
	void close(boost::system::error_code& ec) { acc.close(ec); }

	void accept(socket& socket, endpoint& ep, error_code& ec) {
		acc.accept(socket.get_socket(), ep, ec);
	}

	socket_ptr accept(endpoint& ep, error_code& ec) {
		socket_ptr socket = std::make_shared<socket_ptr::element_type>(acc.get_io_service());
		acc.accept(socket->get_socket(), ep, ec);
		return socket;
	}

	template<typename F>
	void async_accept(socket_ptr socket, F &&f) {
		run = true;
		acc.async_accept(
			 socket->get_socket()
			,[this, socket, f](const boost::system::error_code& ec) {
					f(socket, (!ec ? socket->get_socket().remote_endpoint() : endpoint()), ec);
			 }
		);
	}

	template<typename F>
	void async_accept(F &&handler) {
		socket_ptr socket(new socket_ptr::element_type(acc.get_io_service()));
		async_accept(socket, handler);
	}

	void async_accept(void(*f)(socket_ptr, const endpoint&, const error_code&)) {
		async_accept(
			[f](socket_ptr socket, const endpoint &ep,const error_code &ec) {
				f(socket, ep, ec);
			}
		);
	}

	template<typename Obj>
	void async_accept(Obj* o, void(Obj::*m)(socket_ptr, const endpoint&, const error_code&)) {
		async_accept(
			[o, m](socket_ptr socket, const endpoint &ep,const error_code &ec) {
				(o->*m)(socket, ep, ec);
			}
		);
	}

	template<typename Obj>
	void async_accept(std::shared_ptr<Obj> o, void(Obj::*m)(socket_ptr, const endpoint&, const error_code&)) {
		async_accept(
			[o, m](socket_ptr socket, const endpoint &ep,const error_code &ec) {
				(o.get()->*m)(socket, ep, ec);
			}
		);
	}

	void stop_accept() {
		run = false;
		acc.cancel();
	}
	void stop_accept(error_code& ec) {
		run = false;
		acc.cancel(ec);
	}

private:
	boost::asio::ip::tcp::acceptor acc;
	bool run;
};

/***************************************************************************/

} // namespace easynet

#endif // _easynet__acceptor_hpp
