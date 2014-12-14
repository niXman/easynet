
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

#include <easynet/socket.hpp>
#include "../tests_config.hpp"

#include <iostream>
#include <boost/format.hpp>

/***************************************************************************/

struct client_impl {
	client_impl(boost::asio::io_service& ios, const std::string& ip, boost::uint16_t port)
		:socket(ios)
	{
		socket.connect(ip, port);
	}

	void write() {
		easynet::shared_buffer buf = easynet::buffer_alloc(tests_config::buffer_size);
		strcpy(easynet::buffer_data(buf), "data string");

		socket.async_write(buf, this, &client_impl::write_handler);
	}
	void read() {
		socket.async_read(tests_config::buffer_size, this, &client_impl::read_handler);
	}

	void write_handler(const easynet::error_code& e, easynet::shared_buffer buf, size_t wr) {
		std::cout
		<< boost::format("write_handler(): %2%, %1%, ec = %3%")
			% easynet::buffer_data(buf)
			% wr
			% e.message().c_str()
		<< std::endl;
	}

	void read_handler(const easynet::error_code& e, easynet::shared_buffer buf, size_t rd) {
		std::cout
		<< boost::format("read_handler(): %1%, %2%, ec = %3%")
			% easynet::buffer_data(buf)
			% rd
			% e.message().c_str()
		<< std::endl;
	}

private:
	easynet::socket socket;
};

/***************************************************************************/

int main(int, char**) {
	try {
		boost::asio::io_service ios;
		client_impl client(ios, tests_config::ip, tests_config::port);
		client.write();
		client.read();
		ios.run();
	} catch(const std::exception &ex) {
		std::cout << boost::format("[exception]: %1%") % ex.what() << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

/***************************************************************************/
