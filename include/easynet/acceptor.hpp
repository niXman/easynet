
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
    acceptor(acceptor &&) = default;
    acceptor& operator= (acceptor &&) = default;

    acceptor(boost::asio::io_context& ios, const std::string& ip, std::uint16_t port);

    boost::asio::io_context& get_io_context();

    void cancel();
    void cancel(error_code& ec);
    void close();
    void close(error_code& ec);

    void stop_accept();
    void stop_accept(error_code& ec);

    void accept(socket &sock, error_code& ec, endpoint *ep = nullptr);
    socket accept(error_code& ec, endpoint *ep = nullptr);

    template<typename F>
    void async_accept(socket sock, F f) {
        p_async_accept(
             std::move(sock)
            ,[f=std::move(f)](socket sock, const error_code& ec) mutable
             { f(std::move(sock), sock.remote_endpoint(), ec); }
        );
    }

    template<typename F>
    void async_accept(F f) {
        socket sock(get_io_context());
        async_accept(std::move(sock), std::move(f));
    }

    template<typename Obj>
    void async_accept(Obj* o, void(Obj::*m)(socket, const endpoint&, const error_code&)) {
        async_accept(
            [o, m](socket sock, const endpoint &ep,const error_code &ec)
            { (o->*m)(std::move(sock), ep, ec); }
        );
    }

    template<typename Obj>
    void async_accept(std::shared_ptr<Obj> o, void(Obj::*m)(socket, const endpoint&, const error_code&)) {
        async_accept(
            [o=std::move(o), m](socket sock, const endpoint &ep,const error_code &ec)
            { (o.get()->*m)(std::move(sock), ep, ec); }
        );
    }

private:
    using accept_cb = std::function<void(socket, const error_code &ec)>;
    void p_async_accept(socket sock, accept_cb cb);

private:
    struct impl;
    std::shared_ptr<impl> pimpl;
};

/***************************************************************************/

} // namespace easynet

#endif // __easynet__acceptor_hpp
