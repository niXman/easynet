
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
#include "../tests_config.hpp"

#include <boost/asio/io_context.hpp>

#include <iostream>
#include <cinttypes>

/***************************************************************************/

struct client_impl: std::enable_shared_from_this<client_impl> {
    client_impl(boost::asio::io_context& ios, const char *ip, std::uint16_t port)
        :socket{ios}
        ,m_ip{ip}
        ,m_port{port}
        ,m_idx{}
        ,m_wbuf{}
    {}

    void start() {
        auto self = shared_from_this();
        socket.async_connect(
             m_ip
            ,m_port
            ,[this]
             (const easynet::error_code &ec, easynet::impl_holder holder)
             { on_connected(ec, std::move(holder)); }
            ,std::move(self)
        );
    }

    void on_connected(const easynet::error_code &ec, easynet::impl_holder holder) {
        if ( ec ) {
            std::cout << "can't connect with error \"" << ec.message() << "\"" << std::endl;

            return;
        }

        write(std::move(holder));
    }


    void write(easynet::impl_holder holder) {
        m_wbuf = easynet::buffer_alloc(tests_config::buffer_size);
        std::snprintf(
             easynet::buffer_data(m_wbuf)
            ,easynet::buffer_size(m_wbuf)
            ,"data string no=%" PRIu64 "\n"
            ,static_cast<std::uint64_t>(m_idx)
        );

        ++m_idx;

        socket.async_write_some(m_wbuf, this, &client_impl::write_handler, std::move(holder));
    }
    void write_handler(const easynet::error_code &ec, easynet::shared_buffer buf, size_t wr, easynet::impl_holder holder) {
        std::cout
        << "write_handler(): " << easynet::buffer_data(buf) << ", " << wr << ", ec = " << ec
        << std::endl;

        if ( !ec ) {
            if ( wr != tests_config::buffer_size ) {
                easynet::shared_buffer nbuf = easynet::buffer_lshift(buf, wr);
                socket.async_write_some(std::move(nbuf), this, &client_impl::write_handler, std::move(holder));
            } else {
                socket.async_read_until(tests_config::buffer_size, '\n', this, &client_impl::read_handler, std::move(holder));
            }
        } else {
            std::cout << "[1] ec = " << ec << std::endl;
        }
    }

    void read_handler(const easynet::error_code &ec, easynet::shared_buffer buf, size_t rd, easynet::impl_holder holder) {
        std::cout
        << "read_handler(): " << easynet::buffer_data(buf) << ", " << rd << ", ec = " << ec
        << std::endl;

        if ( !ec ) {
            std::cout << easynet::buffer_data_unshifted(buf) << std::endl;

            assert(std::memcmp(easynet::buffer_data_unshifted(buf), easynet::buffer_data_unshifted(m_wbuf), easynet::buffer_size(buf)) == 0);

            write(std::move(holder));
        } else {
            std::cout << "[2] ec = " << ec << std::endl;
        }
    }

private:
    easynet::socket socket;
    const char *m_ip;
    const std::uint16_t m_port;
    std::size_t m_idx;
    easynet::shared_buffer m_wbuf;
};

/***************************************************************************/

int main(int, char**) {
    try {
        boost::asio::io_context ios;

        {
            auto client = std::make_shared<client_impl>(ios, tests_config::ip, tests_config::port);
            client->start();
        }

        ios.run();
    } catch (const std::exception &ex) {
        std::cout << "[exception]: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

/***************************************************************************/
