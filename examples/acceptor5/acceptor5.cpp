
// Copyright (c) 2013-2019 niXman (github dotty nixman doggy pm dotty me)
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

struct session: std::enable_shared_from_this<session> {
    session(easynet::socket sock)
        :socket(std::move(sock))
    {}
    virtual ~session() {}

    void start() {
        socket.async_read(tests_config::buffer_size, shared_from_this(), &session::read_handler);
    }

    void read_handler(const boost::system::error_code& ec, easynet::shared_buffer buf, size_t rd) {
        std::cout
        << "read_handler: ec = " << ec << ", buf = " << easynet::buffer_data(buf) << ", rd = " << rd
        << std::endl;

        if ( !ec ) {
            socket.async_write(buf, shared_from_this(), &session::write_handler);
        } else {
            std::cout << "[2] ec = " << ec << std::endl;
        }
    }
    void write_handler(const boost::system::error_code& ec, easynet::shared_buffer buf, size_t wr) {
        std::cout
        << "write_handler: ec = " << ec << ", buf = " << easynet::buffer_data(buf) << ", wr = " << wr
        << std::endl;

        if ( !ec ) {
            start();
        } else {
            std::cout << "[3] ec = " << ec << std::endl;
        }
    }

private:
    easynet::socket socket;
};

/***************************************************************************/

struct server {
    server(boost::asio::io_context& ios, const char* ip, boost::uint16_t port)
        :acceptor(ios, ip, port)
    {}

    void start() {
        acceptor.async_accept(this, &server::on_accept);
    }

    void on_accept(easynet::socket socket, const easynet::endpoint &ep, const easynet::error_code &ec) {
        std::cout << "new connection from " << ep << ", ec = " << ec << std::endl;
        if ( !ec ) {
            auto s = std::make_shared<session>(std::move(socket));
            s->start();
        } else {
            std::cout << "[1] ec = " << ec << std::endl;
        }
    }

private:
    easynet::acceptor acceptor;
};

/***************************************************************************/

int main() {
    try {
        boost::asio::io_context ios;
        server server(ios, tests_config::ip, tests_config::port);
        server.start();

        ios.run();
    } catch (const std::exception &ex) {
        std::cout << "[exception]: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

/***************************************************************************/
