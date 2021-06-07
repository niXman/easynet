
// Copyright (c) 2013-2021 niXman (github dotty nixman doggy pm dotty me)
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

#include <boost/asio/io_context.hpp>

#include <iostream>

/***************************************************************************/

struct server {
    server(boost::asio::io_context& ios, const char* ip, boost::uint16_t port)
        :acceptor(ios, ip, port)
    {
        acceptor.async_accept(this, &server::on_accept);
    }

    void on_accept(const easynet::error_code &ec, easynet::socket socket, const easynet::endpoint& ep) {
        char buf[tests_config::buffer_size] = "\0";

        std::cout << "new connection from " << ep << ", ec = " << ec << std::endl;
        if ( !ec ) {
            boost::system::error_code ec;
            auto n = socket.read(buf, tests_config::buffer_size, ec);
            std::cout << "read(): n=" << n << std::endl;
            if ( !ec ) {
                auto n = socket.write(buf, tests_config::buffer_size, ec);
                std::cout << "write(): n=" << n << std::endl;
            } else {
                std::cout << "[1] ec = " << ec << std::endl;
            }
        } else {
            std::cout << "[2] ec = " << ec << std::endl;
        }
    }

private:
    easynet::acceptor acceptor;
};

/***************************************************************************/

int main(int, char**) {
    try {
        boost::asio::io_context ios;

        server server(ios, tests_config::ip, tests_config::port);

        ios.run();
    } catch (const std::exception &ex) {
        std::cout << "[exception]: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

/***************************************************************************/
