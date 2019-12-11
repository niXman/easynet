
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

#ifndef __easynet__acceptor_hpp
#define __easynet__acceptor_hpp

#include <easynet/socket.hpp>

#include <memory>
#include <functional>

namespace easynet {

/***************************************************************************/

struct acceptor {
    acceptor(const acceptor &) = delete;
    acceptor& operator= (const acceptor &) = delete;

    acceptor(acceptor &&);
    acceptor& operator= (acceptor &&);

    acceptor(boost::asio::io_context& ios);
    acceptor(boost::asio::io_context& ios, const char *ip, std::uint16_t port);
    virtual ~acceptor();

    boost::asio::io_context& get_io_context();

    void cancel();
    void cancel(error_code& ec);
    void close();
    void close(error_code& ec);

    void stop_accept();
    void stop_accept(error_code& ec);

    void open(const char *ip, std::uint16_t port);
    void open(const char *ip, std::uint16_t port, error_code& ec);

    void accept(socket &sock, error_code& ec, endpoint *ep = nullptr);
    socket accept(error_code& ec, endpoint *ep = nullptr);

    template<typename F>
    auto async_accept(socket sock, F f, impl_holder holder = {})
        -> decltype(f(error_code{}, std::move(sock), sock.remote_endpoint(), impl_holder{}), void())
    {
        p_async_accept(std::move(sock), std::move(f), std::move(holder));
    }
    template<typename F>
    auto async_accept(socket sock, F f, impl_holder holder = {})
        -> decltype(f(error_code{}, std::move(sock), sock.remote_endpoint()), void())
    {
        p_async_accept(
             std::move(sock)
            ,[f=std::move(f)]
             (const error_code &ec, socket sock, const endpoint &ep, impl_holder)
             { f(ec, std::move(sock), ep); }
            ,std::move(holder)
        );
    }

    template<typename F>
    void async_accept(F f, impl_holder holder = {}) {
        socket sock(get_io_context());
        async_accept(std::move(sock), std::move(f), std::move(holder));
    }

    template<typename Obj>
    void async_accept(Obj* o, void(Obj::*m)(const error_code &, socket, const endpoint &, impl_holder), impl_holder holder = {}) {
        socket sock(get_io_context());
        p_async_accept(
             std::move(sock)
            ,[o, m]
             (const error_code &ec, socket sock, const endpoint &ep, impl_holder holder)
             { (o->*m)(ec, std::move(sock), ep, std::move(holder)); }
            ,std::move(holder)
        );
    }
    template<typename Obj>
    void async_accept(Obj* o, void(Obj::*m)(const error_code &, socket, const endpoint &), impl_holder holder = {}) {
        socket sock(get_io_context());
        p_async_accept(
             std::move(sock)
            ,[o, m]
             (const error_code &ec, socket sock, const endpoint &ep, impl_holder)
             { (o->*m)(ec, std::move(sock), ep); }
            ,std::move(holder)
        );
    }

private:
    using accept_cb = std::function<void(const error_code &, socket, const endpoint &, impl_holder)>;
    void p_async_accept(socket sock, accept_cb cb, impl_holder holder);

private:
    struct impl;
    std::unique_ptr<impl> pimpl;
};

/***************************************************************************/

} // namespace easynet

#endif // __easynet__acceptor_hpp
