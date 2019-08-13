
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

#include <easynet/acceptor.hpp>

#include <boost/asio/ip/tcp.hpp>

namespace easynet {

/***************************************************************************/

struct acceptor::impl {
    impl(boost::asio::io_context& ios, const char *ip, std::uint16_t port)
        :m_acc(ios, boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(ip), port), true)
    {}
    ~impl() {
        error_code ec;
        close(ec);
    }

    void cancel() { m_acc.cancel(); }
    void cancel(boost::system::error_code& ec) { m_acc.cancel(ec); }
    void close() { m_acc.close(); }
    void close(boost::system::error_code& ec) { m_acc.close(ec); }

    void stop_accept() { m_acc.cancel(); }
    void stop_accept(error_code& ec) { m_acc.cancel(ec); }

    void accept(socket &sock, error_code& ec, endpoint* ep) {
        auto *sock_impl = static_cast<boost::asio::ip::tcp::socket*>(sock.get_impl_details());
        boost::asio::ip::tcp::endpoint endpoint;
        m_acc.accept(*sock_impl, endpoint, ec);
        if ( ep && !ec ) {
            ep->ip   = endpoint.address().to_string();
            ep->port = endpoint.port();
        }
    }

    socket accept(error_code& ec, endpoint *ep) {
        socket sock(m_acc.get_io_context());
        auto *sock_impl = static_cast<boost::asio::ip::tcp::socket*>(sock.get_impl_details());
        boost::asio::ip::tcp::endpoint endpoint;
        m_acc.accept(*sock_impl, endpoint, ec);
        if ( ep && !ec ) {
            ep->ip   = endpoint.address().to_string();
            ep->port = endpoint.port();
        }

        return sock;
    }

    void p_async_accept(socket sock, accept_cb cb, impl_holder holder) {
        auto *sock_impl = static_cast<boost::asio::ip::tcp::socket*>(sock.get_impl_details());
        m_acc.async_accept(
             *sock_impl
            ,[sock=std::move(sock), cb=std::move(cb), holder=std::move(holder)]
             (const error_code &ec) mutable
             {
                const auto &ep = sock.remote_endpoint();
                cb(ec, std::move(sock), ep, std::move(holder));
             }
        );
    }

    boost::asio::ip::tcp::acceptor m_acc;
};

/***************************************************************************/

acceptor::acceptor(boost::asio::io_context& ios, const char *ip, boost::uint16_t port)
    :pimpl{std::make_shared<impl>(ios, ip, port)}
{}

acceptor::~acceptor()
{}

/***************************************************************************/

boost::asio::io_context& acceptor::get_io_context() { return pimpl->m_acc.get_io_context(); }

void acceptor::cancel() { return pimpl->cancel(); }
void acceptor::cancel(error_code& ec) { return pimpl->cancel(ec); }
void acceptor::close() { return pimpl->close(); }
void acceptor::close(error_code& ec) { return pimpl->close(ec); }

void acceptor::stop_accept() { return pimpl->stop_accept(); }
void acceptor::stop_accept(error_code& ec) { return pimpl->stop_accept(ec); }

void acceptor::accept(socket &sock, error_code& ec, endpoint* ep) { return pimpl->accept(sock, ec, ep); }
socket acceptor::accept(error_code& ec, endpoint* ep) { return pimpl->accept(ec, ep); }

void acceptor::p_async_accept(socket sock, accept_cb cb, impl_holder holder)
{ return pimpl->p_async_accept(std::move(sock), std::move(cb), std::move(holder)); }

/***************************************************************************/

} // ns easynet
