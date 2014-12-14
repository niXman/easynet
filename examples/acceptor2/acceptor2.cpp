
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
#include <easynet/acceptor.hpp>

#include "../tests_config.hpp"

#include <iostream>
#include <boost/format.hpp>

/***************************************************************************/

int main(int, char**) {
	try {
		boost::asio::io_service ios;
		easynet::acceptor acceptor(ios, tests_config::ip, tests_config::port);

		while ( true ) {
			char buf[tests_config::buffer_size] = "\0";
			easynet::endpoint ep;
			easynet::error_code ec;
			easynet::socket_ptr socket = acceptor.accept(ep, ec);

			std::cout
			<< boost::format("new connection from: %1%, ec = %2%")
				% ep.address().to_string()
				% ec.message()
			<< std::endl;

			if ( !ec ) {
				socket->read(buf, tests_config::buffer_size, ec);
				if ( !ec ) {
					socket->write(buf, tests_config::buffer_size, ec);
				} else {
					std::cout << boost::format("[1] ec = %1%") % ec << std::endl;
				}
			} else {
				std::cout << boost::format("[2] ec = %1%") % ec << std::endl;
			}
		}
	} catch (const std::exception& ex) {
		std::cout << boost::format("[exception]: %1%") % ex.what() << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

/***************************************************************************/
