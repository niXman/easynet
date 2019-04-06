
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

#ifndef __easynet__socket_hpp
#define __easynet__socket_hpp

#include <easynet/shared_buffer.hpp>
#include <easynet/typedefs.hpp>

#include <memory>
#include <functional>

namespace easynet {

/***************************************************************************/

struct socket {
    socket(const socket &) = delete;
    socket& operator= (const socket &) = delete;
    socket(socket &&) = default;
    socket& operator= (socket &&) = default;

    socket(boost::asio::io_context &ios);

    /** returns io_context */
    boost::asio::io_context& get_io_context();

    /** sync connect */
    void connect(const char *ip, std::uint16_t port);
    void connect(const char *ip, std::uint16_t port, error_code &ec);

    /** async connect */
    void async_connect(const char *ip, std::uint16_t port, std::function<void(const error_code &ec)> cb);

    /** disconnect */
    void disconnect();
    void disconnect(error_code &ec);

    /** cancel */
    void cancel();
    void cancel(error_code &ec);

    /** close */
    void close();
    void close(error_code &ec);

    /** reset */
    void reset();
    void reset(error_code &ec);

    /** check for open */
    bool is_open() const;

    /** returns endpoint */
    endpoint local_endpoint() const;
    endpoint local_endpoint(error_code &ec) const;
    endpoint remote_endpoint() const;
    endpoint remote_endpoint(error_code &ec) const;

    /** returns the num of bytes available for read */
    std::size_t available() const;
    std::size_t available(error_code &ec) const;

    /** returns the num of items in read/write queue */
    std::size_t read_queue_size() const;
    std::size_t write_queue_size() const;

    /** clear the read/write/ queue */
    void clear_read_queue();
    void clear_write_queue();

    /** sync write */
    std::size_t write(const void* ptr, std::size_t size);
    std::size_t write(const void* ptr, std::size_t size, error_code &ec);
    std::size_t write(const shared_buffer &buf);
    std::size_t write(const shared_buffer &buf, error_code &ec);

    using handler_type = std::function<void(const error_code&, shared_buffer, std::size_t)>;

    template<typename Obj>
    struct memfn_ptr {
        using type = void(Obj::*)(const error_code&, shared_buffer, std::size_t);
    };

    /** async write */
    template<typename F>
    void async_write(shared_buffer buf, F handler) { append_write_task(std::move(buf), std::move(handler)); }
    template<typename Obj>
    void async_write(shared_buffer buf, Obj* o, typename memfn_ptr<Obj>::type m) {
        async_write(
             std::move(buf)
            ,[o, m](const error_code &ec, shared_buffer buf, std::size_t wr)
             { (o->*m)(ec, std::move(buf), wr); }
        );
    }
    template<typename Obj>
    void async_write(shared_buffer buf, std::shared_ptr<Obj> o, typename memfn_ptr<Obj>::type m) {
        async_write(
             std::move(buf)
            ,[o=std::move(o), m](const error_code &ec, shared_buffer buf, std::size_t wr)
             { (o.get()->*m)(ec, std::move(buf), wr); }
        );
    }

    /** sync write some */
    std::size_t write_some(const void* ptr, std::size_t size);
    std::size_t write_some(const void* ptr, std::size_t size, error_code &ec);
    std::size_t write_some(const shared_buffer &buf);
    std::size_t write_some(const shared_buffer &buf, error_code &ec);

    /** async write some */
    template<typename F>
    void async_write_some(shared_buffer buf, F handler) { append_write_some_task(std::move(buf), std::move(handler));}
    template<typename Obj>
    void async_write_some(shared_buffer buf, Obj* o, typename memfn_ptr<Obj>::type m) {
        async_write_some(
             std::move(buf)
            ,[o, m](const error_code &ec, shared_buffer buf, std::size_t wr)
             { (o->*m)(ec, std::move(buf), wr); }
        );
    }
    template<typename Obj>
    void async_write_some(shared_buffer buf, std::shared_ptr<Obj> o, typename memfn_ptr<Obj>::type m) {
        async_write_some(
             std::move(buf)
            ,[o=std::move(o), m](const error_code &ec, shared_buffer buf, std::size_t wr)
             { (o.get()->*m)(ec, std::move(buf), wr); }
        );
    }

    /** sync read */
    std::size_t read(void* ptr, std::size_t size);
    std::size_t read(void* ptr, std::size_t size, error_code &ec);
    shared_buffer read(std::size_t size);
    shared_buffer read(std::size_t size, error_code &ec);

    /** async read */
    template<typename F>
    void async_read(std::size_t size, F handler) { append_read_task(shared_buffer{nullptr, size}, std::move(handler)); }
    template<typename F>
    void async_read(shared_buffer buf, F handler) { append_read_task(std::move(buf), std::move(handler)); }
    template<typename Obj>
    void async_read(std::size_t size, Obj* o, typename memfn_ptr<Obj>::type m) {
        async_read(
             size
            ,[o, m](const error_code &ec, shared_buffer buf, std::size_t rd)
             { (o->*m)(ec, std::move(buf), rd); }
        );
    }
    template<typename Obj>
    void async_read(shared_buffer buf, Obj* o, typename memfn_ptr<Obj>::type m) {
        async_read(
             std::move(buf)
            ,[o, m](const error_code &ec, shared_buffer buf, std::size_t rd)
             { (o->*m)(ec, std::move(buf), rd); }
        );
    }
    template<typename Obj>
    void async_read(std::size_t size, std::shared_ptr<Obj> o, typename memfn_ptr<Obj>::type m) {
        async_read(
             size
            ,[o=std::move(o), m](const error_code &ec, shared_buffer buf, std::size_t rd)
             { (o.get()->*m)(ec, std::move(buf), rd); }
        );
    }
    template<typename Obj>
    void async_read(shared_buffer buf, std::shared_ptr<Obj> o, typename memfn_ptr<Obj>::type m) {
        async_read(
             std::move(buf)
            ,[o=std::move(o), m](const error_code &ec, shared_buffer buf, std::size_t rd)
             { (o.get()->*m)(ec, std::move(buf), rd); }
        );
    }

    /** sync read some */
    std::size_t read_some(void* ptr, std::size_t size);
    std::size_t read_some(void* ptr, std::size_t size, error_code &ec);
    shared_buffer read_some(std::size_t size);
    shared_buffer read_some(std::size_t size, error_code &ec);

    /** async read some */
    template<typename F>
    void async_read_some(std::size_t size, F handler) { append_read_some_task(shared_buffer{nullptr, size}, std::move(handler)); }
    template<typename F>
    void async_read_some(shared_buffer buf, F handler) { append_read_some_task(std::move(buf), std::move(handler)); }
    template<typename Obj>
    void async_read_some(std::size_t size, Obj* o, typename memfn_ptr<Obj>::type m) {
        async_read_some(
             size
            ,[o, m](const error_code &ec, shared_buffer buf, std::size_t rd)
             { (o->*m)(ec, std::move(buf), rd); }
        );
    }
    template<typename Obj>
    void async_read_some(shared_buffer buf, Obj* o, typename memfn_ptr<Obj>::type m) {
        async_read_some(
             std::move(buf)
            ,[o, m](const error_code &ec, shared_buffer buf, std::size_t rd)
             { (o->*m)(ec, std::move(buf), rd); }
        );
    }
    template<typename Obj>
    void async_read_some(std::size_t size, std::shared_ptr<Obj> o, typename memfn_ptr<Obj>::type m) {
        async_read_some(
             size
            ,[o=std::move(o), m](const error_code &ec, shared_buffer buf, std::size_t rd)
             { (o.get()->*m)(ec, std::move(buf), rd); }
        );
    }
    template<typename Obj>
    void async_read_some(shared_buffer buf, std::shared_ptr<Obj> o, typename memfn_ptr<Obj>::type m) {
        async_read_some(
             std::move(buf)
            ,[o=std::move(o), m](const error_code &ec, shared_buffer buf, std::size_t rd)
             { (o.get()->*m)(ec, std::move(buf), rd); }
        );
    }

private:
    void append_write_task(shared_buffer buf, handler_type cb);
    void append_write_some_task(shared_buffer buf, handler_type cb);
    void append_read_task(shared_buffer buf, handler_type cb);
    void append_read_some_task(shared_buffer buf, handler_type cb);

    friend struct acceptor;

    /** returns the pointer to impl details */
    void* get_impl_details();

private:
    struct impl;
    std::shared_ptr<impl> pimpl;
};

/***************************************************************************/

} // namespace easynet

#endif // __easynet__socket_hpp
