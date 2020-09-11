
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

#ifndef __easynet__socket_hpp
#define __easynet__socket_hpp

#include <easynet/shared_buffer.hpp>
#include <easynet/typedefs.hpp>

#include <functional>

namespace easynet {

using opid = void *;

/***************************************************************************/

struct socket {
    socket(const socket &) = delete;
    socket& operator= (const socket &) = delete;

    socket(socket &&r) noexcept;
    socket& operator= (socket &&r) noexcept;

    socket(boost::asio::io_context &ios);
    virtual ~socket();

    /** returns io_context */
    boost::asio::io_context& get_io_context();

    /** sync connect */
    void connect(const char *ip, std::uint16_t port);
    void connect(const char *ip, std::uint16_t port, error_code &ec);

    /** async connect */
    using connect_cb_type = std::function<void(const error_code &ec, impl_holder)>;
    void async_connect(const char *ip, std::uint16_t port, connect_cb_type cb, impl_holder holder = {});

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

    /** pop front/back for read queue*/
    void read_queue_pop_front();
    void read_queue_pop_front(std::size_t n);
    void read_queue_pop_back();
    void read_queue_pop_back(std::size_t n);

    /** pop front/back for read queue*/
    void write_queue_pop_front();
    void write_queue_pop_front(std::size_t n);
    void write_queue_pop_back();
    void write_queue_pop_back(std::size_t n);

    /** wait for the socket to become ready to write */
    void wait_write();
    void wait_write(error_code &ec);

    /** async wait for the socket to become ready to write */
    template<typename F>
    void async_wait_write(F f, impl_holder holder = {}) {
        append_task(e_task::wait_write, std::move(f), std::move(holder));
    }
    template<typename Obj>
    void async_wait_write(Obj *o, void(Obj::*m)(const error_code &, impl_holder), impl_holder holder = {}) {
        append_task(
             e_task::wait_write
            ,[o, m]
             (const error_code &ec, impl_holder holder)
             { (o->*m)(ec, std::move(holder)); }
            ,std::move(holder)
        );
    }

    /** async wait for the socket to become ready to read */
    template<typename F>
    void async_wait_read(F f, impl_holder holder = {}) {
        append_task(e_task::wait_read, std::move(f), std::move(holder));
    }
    template<typename Obj>
    void async_wait_read(Obj *o, void(Obj::*m)(const error_code &, impl_holder), impl_holder holder = {}) {
        append_task(
             e_task::wait_read
            ,[o, m]
             (const error_code &ec, impl_holder holder)
             { (o->*m)(ec, std::move(holder)); }
            ,std::move(holder)
        );
    }

    /** async wait for a socket to have error conditions pending */
    template<typename F>
    void async_wait_error(F f, impl_holder holder = {}) {
        append_task(e_task::wait_error, std::move(f), std::move(holder));
    }
    template<typename Obj>
    void async_wait_error(Obj *o, void(Obj::*m)(const error_code &, impl_holder), impl_holder holder = {}) {
        append_task(
             e_task::wait_error
            ,[o, m]
             (const error_code &ec, impl_holder holder)
             { (o->*m)(ec, std::move(holder)); }
            ,std::move(holder)
        );
    }

    /** sync write */
    std::size_t write(const void* ptr, std::size_t size);
    std::size_t write(const void* ptr, std::size_t size, error_code &ec);
    std::size_t write(const shared_buffer &buf);
    std::size_t write(const shared_buffer &buf, error_code &ec);

    template<typename Obj>
    struct memfn_ptr {
        using type = void(Obj::*)(const error_code&, shared_buffer, std::size_t, impl_holder);
    };

    /** async write */
    template<typename F>
    void async_write(shared_buffer buf, F f, impl_holder holder = {}) {
        append_task(e_task::write, std::move(buf), std::move(f), std::move(holder));
    }
    template<typename F>
    void async_write(const void *ptr, std::size_t size, F f, impl_holder holder = {}) {
        shared_buffer buf{
            {const_cast<char *>(static_cast<const char *>(ptr)), [](char*){}}
            ,size
            ,0
        };
        append_task(e_task::write, std::move(buf), std::move(f), std::move(holder));
    }
    template<typename Obj>
    void async_write(const void *ptr, std::size_t size, Obj* o, typename memfn_ptr<Obj>::type m, impl_holder holder = {}) {
        async_write(
             ptr
            ,size
            ,[o, m](const error_code &ec, shared_buffer buf, std::size_t wr, impl_holder holder)
             { (o->*m)(ec, std::move(buf), wr, std::move(holder)); }
            ,std::move(holder)
        );
    }
    template<typename Obj>
    void async_write(shared_buffer buf, Obj* o, typename memfn_ptr<Obj>::type m, impl_holder holder = {}) {
        async_write(
             std::move(buf)
            ,[o, m](const error_code &ec, shared_buffer buf, std::size_t wr, impl_holder holder)
             { (o->*m)(ec, std::move(buf), wr, std::move(holder)); }
            ,std::move(holder)
        );
    }

    /** sync write some */
    std::size_t write_some(const void* ptr, std::size_t size);
    std::size_t write_some(const void* ptr, std::size_t size, error_code &ec);
    std::size_t write_some(const shared_buffer &buf);
    std::size_t write_some(const shared_buffer &buf, error_code &ec);

    /** async write some */
    template<typename F>
    void async_write_some(shared_buffer buf, F f, impl_holder holder = {}) {
        append_task(e_task::write_some, std::move(buf), std::move(f), std::move(holder));
    }
    template<typename F>
    void async_write_some(const void *ptr, std::size_t size, F f, impl_holder holder = {}) {
        shared_buffer buf{
             {const_cast<char *>(static_cast<const char *>(ptr)), [](char*){}}
            ,size
            ,0
        };
        append_task(e_task::write_some, std::move(buf), std::move(f), std::move(holder));
    }
    template<typename Obj>
    void async_write_some(const void *ptr, std::size_t size, Obj* o, typename memfn_ptr<Obj>::type m, impl_holder holder = {}) {
        async_write_some(
             ptr
            ,size
            ,[o, m](const error_code &ec, shared_buffer buf, std::size_t wr, impl_holder holder)
             { (o->*m)(ec, std::move(buf), wr, std::move(holder)); }
            ,std::move(holder)
        );
    }
    template<typename Obj>
    void async_write_some(shared_buffer buf, Obj* o, typename memfn_ptr<Obj>::type m, impl_holder holder = {}) {
        async_write_some(
             std::move(buf)
            ,[o, m](const error_code &ec, shared_buffer buf, std::size_t wr, impl_holder holder)
             { (o->*m)(ec, std::move(buf), wr, std::move(holder)); }
            ,std::move(holder)
        );
    }

    /** wait for the socket to become ready to read */
    void wait_read();
    void wait_read(error_code &ec);

    /** sync read */
    std::size_t read(void* ptr, std::size_t size);
    std::size_t read(void* ptr, std::size_t size, error_code &ec);
    shared_buffer read(std::size_t size);
    shared_buffer read(std::size_t size, error_code &ec);

    /** async read */
    template<typename F>
    void async_read(std::size_t size, F f, impl_holder holder = {}) {
        append_task(e_task::read, shared_buffer{nullptr, size, 0}, std::move(f), std::move(holder));
    }
    template<typename F>
    void async_read(void *ptr, std::size_t size, F f, impl_holder holder = {}) {
        shared_buffer buf{
             {static_cast<char *>(ptr), [](char *){}}
            ,size
            ,0
        };
        append_task(e_task::read, std::move(buf), std::move(f), std::move(holder));
    }
    template<typename F>
    void async_read(shared_buffer buf, F f, impl_holder holder = {}) {
        append_task(e_task::read, std::move(buf), std::move(f), std::move(holder));
    }
    template<typename Obj>
    void async_read(void *ptr, std::size_t size, Obj* o, typename memfn_ptr<Obj>::type m, impl_holder holder = {}) {
        async_read(
             ptr
            ,size
            ,[o, m](const error_code &ec, shared_buffer buf, std::size_t rd, impl_holder holder)
             { (o->*m)(ec, std::move(buf), rd, std::move(holder)); }
            ,std::move(holder)
        );
    }
    template<typename Obj>
    void async_read(std::size_t size, Obj* o, typename memfn_ptr<Obj>::type m, impl_holder holder = {}) {
        async_read(
             size
            ,[o, m](const error_code &ec, shared_buffer buf, std::size_t rd, impl_holder holder)
             { (o->*m)(ec, std::move(buf), rd, std::move(holder)); }
            ,std::move(holder)
        );
    }
    template<typename Obj>
    void async_read(shared_buffer buf, Obj* o, typename memfn_ptr<Obj>::type m, impl_holder holder = {}) {
        async_read(
             std::move(buf)
            ,[o, m](const error_code &ec, shared_buffer buf, std::size_t rd, impl_holder holder)
             { (o->*m)(ec, std::move(buf), rd, std::move(holder)); }
            ,std::move(holder)
        );
    }

    /** sync read some */
    std::size_t read_some(void* ptr, std::size_t size);
    std::size_t read_some(void* ptr, std::size_t size, error_code &ec);
    shared_buffer read_some(std::size_t size);
    shared_buffer read_some(std::size_t size, error_code &ec);

    /** async read some */
    template<typename F>
    void async_read_some(std::size_t size, F f, impl_holder holder = {}) {
        append_task(e_task::read_some, shared_buffer{nullptr, size, 0}, std::move(f), std::move(holder));
    }
    template<typename F>
    void async_read_some(void *ptr, std::size_t size, F f, impl_holder holder = {}) {
        shared_buffer buf{
             {static_cast<char *>(ptr), [](char *){}}
            ,size
            ,0
        };
        append_task(e_task::read_some, std::move(buf), std::move(f), std::move(holder));
    }
    template<typename F>
    void async_read_some(shared_buffer buf, F f, impl_holder holder = {}) {
        append_task(e_task::read_some, std::move(buf), std::move(f), std::move(holder));
    }
    template<typename Obj>
    void async_read_some(std::size_t size, Obj* o, typename memfn_ptr<Obj>::type m, impl_holder holder = {}) {
        async_read_some(
             size
            ,[o, m](const error_code &ec, shared_buffer buf, std::size_t rd, impl_holder holder)
             { (o->*m)(ec, std::move(buf), rd, std::move(holder)); }
            ,std::move(holder)
        );
    }
    template<typename Obj>
    void async_read_some(void *ptr, std::size_t size, Obj* o, typename memfn_ptr<Obj>::type m, impl_holder holder = {}) {
        async_read_some(
             ptr
            ,size
            ,[o, m](const error_code &ec, shared_buffer buf, std::size_t rd, impl_holder holder)
             { (o->*m)(ec, std::move(buf), rd, std::move(holder)); }
            ,std::move(holder)
        );
    }
    template<typename Obj>
    void async_read_some(shared_buffer buf, Obj* o, typename memfn_ptr<Obj>::type m, impl_holder holder = {}) {
        async_read_some(
             std::move(buf)
            ,[o, m](const error_code &ec, shared_buffer buf, std::size_t rd, impl_holder holder)
             { (o->*m)(ec, std::move(buf), rd, std::move(holder)); }
            ,std::move(holder)
        );
    }

private:
    enum class e_task: std::uint8_t {
         write
        ,write_some
        ,wait_write
        ,read
        ,read_some
        ,wait_read
        ,wait_error
    };

    using handler_type = std::function<void(const error_code&, shared_buffer, std::size_t, impl_holder)>;
    void append_task(e_task task, shared_buffer buf, handler_type cb, impl_holder holder);
    using handler_type2 = std::function<void(const error_code&, impl_holder)>;
    void append_task(e_task task, handler_type2 ch, impl_holder holder);

    friend struct acceptor;
    /** returns the pointer to impl details */
    void* get_impl_details();

private:
    struct impl;
    std::unique_ptr<impl> pimpl;
};

/***************************************************************************/

} // namespace easynet

#endif // __easynet__socket_hpp
