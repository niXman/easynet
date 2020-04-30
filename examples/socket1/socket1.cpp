
// Copyright (c) 2013-2020 niXman (github dotty nixman doggy pm dotty me)
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

#undef NDEBUG

#include <easynet/socket.hpp>
#include "../tests_config.hpp"

#include <boost/asio/io_context.hpp>

#include <map>
#include <iostream>

std::map<std::size_t, std::string> wmap, rmap;
std::size_t widx = 0, ridx = 0;

/***************************************************************************/

void write_handler(const boost::system::error_code& ec, easynet::shared_buffer buf, std::size_t wr, easynet::impl_holder) {
    std::cout
    << "write_handler(): data=\"" << easynet::buffer_data(buf) << "\", wr=" << wr << ", ec=" << ec.message()
    << std::endl;

    wmap[widx++] = easynet::buffer_data(buf);
}

void read_handler(const boost::system::error_code& ec, easynet::shared_buffer buf, std::size_t rd, easynet::impl_holder) {
    std::cout
    << "read_handler (): data=\"" << easynet::buffer_data(buf) << "\", rd=" << rd << ", ec=" << ec.message()
    << std::endl;

    rmap[ridx++] = easynet::buffer_data(buf);
}

/***************************************************************************/

int main(int, char**) {
    try {
        boost::asio::io_context ios;
        easynet::socket socket(ios);

        socket.connect(tests_config::ip, tests_config::port);

        for ( auto idx = 0u; idx < tests_config::iterations; ++idx ) {
            easynet::shared_buffer buf = easynet::buffer_alloc(tests_config::buffer_size);
            std::snprintf(easynet::buffer_data(buf), easynet::buffer_size(buf), "%u", idx);

            socket.async_write(std::move(buf), write_handler);
        }

        assert(socket.write_queue_size() == tests_config::iterations);
        socket.write_queue_pop_front();
        assert(socket.write_queue_size() == tests_config::iterations-1);
        socket.write_queue_pop_front(3);
        assert(socket.write_queue_size() == tests_config::iterations-4);

        socket.write_queue_pop_back();
        assert(socket.write_queue_size() == tests_config::iterations-5);
        socket.write_queue_pop_back(3);
        assert(socket.write_queue_size() == tests_config::iterations-8);

        for ( auto idx = 0u; idx < tests_config::iterations; ++idx ) {
            socket.async_read(tests_config::buffer_size, read_handler);
        }

        assert(socket.read_queue_size() == tests_config::iterations);
        socket.read_queue_pop_front();
        assert(socket.read_queue_size() == tests_config::iterations-1);
        socket.read_queue_pop_front(3);
        assert(socket.read_queue_size() == tests_config::iterations-4);

        socket.read_queue_pop_back();
        assert(socket.read_queue_size() == tests_config::iterations-5);
        socket.read_queue_pop_back(3);
        assert(socket.read_queue_size() == tests_config::iterations-8);

        ios.run();

        assert(wmap == rmap);
    } catch(const std::exception &ex) {
        std::cout << "[exception]: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

/***************************************************************************/
