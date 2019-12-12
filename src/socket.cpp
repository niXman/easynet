
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

#include <easynet/shared_buffer.hpp>
#include <easynet/handler_allocator.hpp>
#include <easynet/preallocated_handler.hpp>
#include <easynet/socket.hpp>

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>

#include <boost/intrusive/list.hpp>

#include <cstdio>
#include <cassert>

//#define EASYNET_DEBUG_ADD_REMOVE_TASKS
#ifdef EASYNET_DEBUG_ADD_REMOVE_TASKS
#   define EASYNET_EXPAND_EXPR(...) __VA_ARGS__
#   include <iostream>
#else
#   define EASYNET_EXPAND_EXPR(...)
#endif // EASYNET_DEBUG_ADD_REMOVE_TASKS

namespace easynet {

/***************************************************************************/

#define EASYNET_TRY_CATCH_BLOCK(...) \
    try { \
        __VA_ARGS__; \
    } catch (const std::exception &ex) { \
        std::fprintf(stderr, "this=%p, %s(%d): [std::exception] %s\n", static_cast<const void *>(this), __FILE__, __LINE__, ex.what()); \
    } catch (...) { \
        std::fprintf(stderr, "this=%p, %s(%d): [unknown exception]\n", static_cast<const void *>(this), __FILE__, __LINE__); \
    } \
    std::fflush(stderr);

/***************************************************************************/

struct socket::impl {
    impl(boost::asio::io_context& ios)
        :m_sock{ios}
        ,m_read_queue{}
        ,m_write_queue{}
    {
        EASYNET_EXPAND_EXPR(std::cout << "socket::impl(" << this << ")" << std::endl;)
    }
    ~impl() {
        EASYNET_EXPAND_EXPR(std::cout << "socket::~impl(" << this << ")" << std::endl;)

        boost::system::error_code ec;
        disconnect(ec);
    }

    /** returns io_context */
    boost::asio::io_context& get_io_context() { return m_sock.get_io_context(); }

    /** sync connect */
    void connect(const char *ip, boost::uint16_t port) {
        m_sock.connect(boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(ip), port));
    }
    void connect(const char *ip, boost::uint16_t port, error_code &ec) {
        m_sock.connect(boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(ip), port), ec);
    }

    void async_connect(const char *ip, std::uint16_t port, std::function<void(const error_code &ec, impl_holder)> cb, impl_holder holder) {
        m_sock.async_connect(
             boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(ip), port)
            ,[cb=std::move(cb), holder=std::move(holder)]
             (const error_code &ec)
             { cb(ec, std::move(holder)); }
        );
    }

    void disconnect() {
        reset();
        if ( is_open() ) {
            m_sock.shutdown(boost::asio::ip::tcp::socket::shutdown_both);
            m_sock.close();
        }
    }
    void disconnect(error_code& ec) {
        reset(ec);
        if ( ec ) return;
        if ( is_open() ) {
            m_sock.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
            if ( ec ) return;
            m_sock.close(ec);
        }
    }

    void cancel() { m_sock.cancel(); }
    void cancel(error_code& ec) { m_sock.cancel(ec); }

    void close() { m_sock.close(); }
    void close(error_code& ec) { m_sock.close(ec); }

    void reset() {
        cancel();
        clear_read_queue();
        clear_write_queue();
    }
    void reset(error_code& ec) {
        cancel(ec);
        clear_read_queue();
        clear_write_queue();
    }

    bool is_open() const { return m_sock.is_open(); }

    endpoint local_endpoint() const { auto ep = m_sock.local_endpoint(); return {ep.address().to_string(), ep.port()}; }
    endpoint local_endpoint(error_code &ec) const { auto ep = m_sock.local_endpoint(ec); return {ep.address().to_string(), ep.port()}; }
    endpoint remote_endpoint() const { auto ep = m_sock.remote_endpoint(); return {ep.address().to_string(), ep.port()}; }
    endpoint remote_endpoint(error_code &ec) const { auto ep = m_sock.remote_endpoint(ec); return {ep.address().to_string(), ep.port()}; }

    std::size_t available() const { return m_sock.available(); }
    std::size_t available(error_code& ec) const { return m_sock.available(ec); }

    std::size_t read_queue_size() const { return m_read_queue.size(); }
    std::size_t write_queue_size() const { return m_write_queue.size(); }

    void clear_read_queue() {
        m_read_queue.clear_and_dispose(
            [](task_item *item) {
                if ( !item->in_process ) { delete item; }
            }
        );
    }
    void clear_write_queue() {
        m_write_queue.clear_and_dispose(
            [](task_item *item) {
                if ( !item->in_process ) { delete item; }
            }
        );
    }

    void wait_write() {
        m_sock.wait(boost::asio::ip::tcp::socket::wait_write);
    }
    void wait_write(error_code &ec) {
        m_sock.wait(boost::asio::ip::tcp::socket::wait_write, ec);
    }

    std::size_t write(const void* ptr, std::size_t size) {
        return boost::asio::write(m_sock, boost::asio::buffer(ptr, size));
    }
    std::size_t write(const void* ptr, std::size_t size, error_code& ec) {
        return boost::asio::write(m_sock, boost::asio::buffer(ptr, size), ec);
    }
    std::size_t write(const shared_buffer &buf) {
        return write(buf.data.get(), buf.size);
    }
    std::size_t write(const shared_buffer &buf, error_code& ec) {
        return write(buf.data.get(), buf.size, ec);
    }
    std::size_t write_some(const void* ptr, std::size_t size) {
        return m_sock.write_some(boost::asio::buffer(ptr, size));
    }
    std::size_t write_some(const void* ptr, std::size_t size, error_code& ec) {
        return m_sock.write_some(boost::asio::buffer(ptr, size), ec);
    }
    std::size_t write_some(const shared_buffer &buf) {
        return write_some(buf.data.get(), buf.size);
    }
    std::size_t write_some(const shared_buffer &buf, error_code& ec) {
        return write_some(buf.data.get(), buf.size, ec);
    }

    void wait_read() {
        m_sock.wait(boost::asio::ip::tcp::socket::wait_read);
    }
    void wait_read(error_code &ec) {
        m_sock.wait(boost::asio::ip::tcp::socket::wait_read, ec);
    }

    std::size_t read(void* ptr, std::size_t size) {
        return boost::asio::read(m_sock, boost::asio::buffer(ptr, size));
    }
    std::size_t read(void* ptr, std::size_t size, error_code &ec) {
        return boost::asio::read(m_sock, boost::asio::buffer(ptr, size), ec);
    }
    shared_buffer read(std::size_t size) {
        shared_buffer buf = buffer_alloc(size);
        read(buf.data.get(), buf.size);

        return buf;
    }
    shared_buffer read(std::size_t size, error_code &ec) {
        shared_buffer buf = buffer_alloc(size);
        read(buf.data.get(), buf.size, ec);

        return buf;
    }
    std::size_t read_some(void* ptr, std::size_t size) {
        return m_sock.read_some(boost::asio::buffer(ptr, size));
    }
    std::size_t read_some(void* ptr, std::size_t size, error_code &ec) {
        return m_sock.read_some(boost::asio::buffer(ptr, size), ec);
    }
    shared_buffer read_some(std::size_t size) {
        shared_buffer buf = buffer_alloc(size);
        read_some(buf.data.get(), buf.size);

        return buf;
    }
    shared_buffer read_some(std::size_t size, error_code &ec) {
        shared_buffer buf = buffer_alloc(size);
        read_some(buf.data.get(), buf.size, ec);

        return buf;
    }

    struct task_item {
        EASYNET_EXPAND_EXPR(
            task_item(e_task t, bool inp, shared_buffer b, handler_type c, impl_holder h)
                :task{t}
                ,in_process{inp}
                ,buffer{std::move(b)}
                ,handler{std::move(c)}
                ,holder{std::move(h)}
            { std::cout << "task_item ctor: this=" << this << ", op=" << task_name(task) << ", inp=" << in_process << std::endl; }
            ~task_item()
            { std::cout << "task_item dtor: this=" << this << ", op=" << task_name(task) << ", inp=" << in_process << std::endl; }
        )

        e_task task;
        bool in_process;
        shared_buffer buffer;
        handler_type handler;
        impl_holder holder;

        boost::intrusive::list_member_hook<> m_member_hook{};
    };
    using list_member_hook_options = boost::intrusive::member_hook<
         task_item
        ,boost::intrusive::list_member_hook<>
        ,&task_item::m_member_hook
    >;
    using queue_type = boost::intrusive::list<task_item, list_member_hook_options>;

    EASYNET_EXPAND_EXPR(
        static const char* task_name(socket::e_task task) {
            return task == e_task::read ? "read"
                : task == socket::e_task::read_some ? "read_some"
                    : task == e_task::write ? "write"
                        : task == e_task::write_some ? "write_some"
                            : "UNKNOWN_TASK"
            ;
        }
    )

    void read_queue_pop_front(std::size_t n) {
        auto beg = m_read_queue.begin();
        for ( std::size_t i = 0; i < n && beg != m_read_queue.end(); ) {
            if ( !beg->in_process ) {
                beg = m_read_queue.erase_and_dispose(beg, [](task_item *item){ delete item; });
                ++i;
            } else {
                ++beg;
            }
        }
    }
    void read_queue_pop_back(std::size_t n) {
        auto end = std::prev(m_read_queue.end());
        for ( std::size_t i = 0; i < n && !m_read_queue.empty(); ) {
            if ( !end->in_process ) {
                m_read_queue.pop_back_and_dispose([](task_item *item){ delete item; });
                ++i;
            }
        }
    }

    void write_queue_pop_front(std::size_t n) {
        auto beg = m_write_queue.begin();
        for ( std::size_t i = 0; i < n && beg != m_write_queue.end(); ) {
            if ( !beg->in_process ) {
                beg = m_write_queue.erase_and_dispose(beg, [](task_item *item){ delete item; });
                ++i;
            } else {
                 ++beg;
            }
        }
    }
    void write_queue_pop_back(std::size_t n) {
        auto end = std::prev(m_write_queue.end());
        for ( std::size_t i = 0; i < n && !m_write_queue.empty(); ) {
            if ( !end->in_process ) {
                m_write_queue.pop_back_and_dispose([](task_item *item){ delete item; });
                ++i;
            }
        }
    }

    void append_task(e_task task, handler_type2 cb, impl_holder holder) {
        assert(task == e_task::read || task == e_task::write);

        auto wt = task == e_task::read
            ? boost::asio::ip::tcp::socket::wait_read
            : task == e_task::write
                ? boost::asio::ip::tcp::socket::wait_write
                : boost::asio::ip::tcp::socket::wait_error
        ;

        m_sock.async_wait(
             wt
            ,[cb=std::move(cb), holder=std::move(holder)]
             (const error_code& ec) mutable
             { cb(ec, std::move(holder)); }
        );
    }

    void append_task(socket::e_task task, shared_buffer buf, handler_type cb, impl_holder holder) {
        auto *item = new task_item{task, false, std::move(buf), std::move(cb), std::move(holder)};

        auto &toappend = (task == e_task::read || task == e_task::read_some) ? m_read_queue : m_write_queue;

        EASYNET_EXPAND_EXPR(std::cout << "socket::append_task: this=" << this << ", op=" << task_name(task) << ", size=" << item->buffer.size << ", addr=" << item << ", queue_size=" << toappend.size() << std::endl;)

        toappend.push_back(*item);

        if ( toappend.size() == 1u ) {
            EASYNET_EXPAND_EXPR(std::cout << "socket::append_task->start_task: this=" << this << ", op=" << task_name(task) << ", size=" << item->buffer.size << ", addr=" << item << std::endl;)
            start_task(item);
        }
    }
    void start_task(task_item *item) {
        switch ( item->task ) {
            case e_task::read:       { start_read(item); return; }
            case e_task::read_some:  { start_read_some(item); return; }
            case e_task::write:      { start_write(item); return; }
            case e_task::write_some: { start_write_some(item); return; }
            default: break;
        }
    }

    /**  */
    void start_write(task_item *item) {
        item->in_process = true;
        auto buf_data = buffer_data(item->buffer);
        auto buf_size = buffer_size(item->buffer);

        EASYNET_EXPAND_EXPR(std::cout << "socket::start_write: this=" << this << ", op=" << task_name(item->task) << ", size=" << buf_size << ", addr=" << item << std::endl;)

        boost::asio::async_write(
             m_sock
            ,boost::asio::buffer(buf_data, buf_size)
            ,make_preallocated_handler(
                 m_write_handler_allocator
                ,[this, item]
                 (const error_code& ec, std::size_t wr)
                 { write_handler(item, ec, wr); }
            )
        );
    }
    void start_write_some(task_item *item) {
        item->in_process = true;
        auto buf_data = buffer_data(item->buffer);
        auto buf_size = buffer_size(item->buffer);

        EASYNET_EXPAND_EXPR(std::cout << "socket::start_write_some: this=" << this << ", op=" << task_name(item->task) << ", size=" << buf_size << ", addr=" << item << std::endl;)

        m_sock.async_write_some(
             boost::asio::buffer(buf_data, buf_size)
            ,[this, item]
             (const error_code& ec, std::size_t wr)
             { write_handler(item, ec, wr); }
        );
    }
    void write_handler(task_item *item, const error_code& ec, std::size_t wr) {
        auto holder = std::move(item->holder);
        auto buffer = std::move(item->buffer);
        auto funct  = std::move(item->handler);

        EASYNET_EXPAND_EXPR(
            auto size = buffer.size;
            auto task = task_name(item->task);
            std::cout << "socket::write_handler enter: this=" << this << ", op=" << task << ", size=" << size << ", addr=" << item << std::endl;
        )

        m_write_queue.pop_front_and_dispose([](task_item *item){ delete item; });

        EASYNET_TRY_CATCH_BLOCK(funct(ec, std::move(buffer), wr, std::move(holder));)

        if ( !m_write_queue.empty() && !m_write_queue.front().in_process ) {
            task_item *item2 = &(m_write_queue.front());
            start_task(item2);
        }

        EASYNET_EXPAND_EXPR(std::cout << "socket::write_handler leave: this=" << this << ", op=" << task << ", size=" << size << ", addr=" << item << std::endl;)
    }

    /**  */
    void start_read(task_item *item) {
        item->in_process = true;
        if ( !item->buffer.data ) {
            item->buffer = buffer_alloc(item->buffer.size);
        }

        auto buf_data = buffer_data(item->buffer);
        auto buf_size = buffer_size(item->buffer);

        EASYNET_EXPAND_EXPR(std::cout << "socket::start_read: this=" << this << ", op=" << task_name(item->task) << ", size=" << buf_size << ", addr=" << item << std::endl;)

        boost::asio::async_read(
             m_sock
            ,boost::asio::buffer(buf_data, buf_size)
            ,make_preallocated_handler(
                 m_read_handler_allocator
                ,[this, item]
                 (const error_code &ec, std::size_t rd)
                 { read_handler(item, ec, rd); }
            )
        );
    }
    void start_read_some(task_item *item) {
        item->in_process = true;
        if ( !item->buffer.data ) {
            item->buffer = buffer_alloc(item->buffer.size);
        }

        auto buf_data = buffer_data(item->buffer);
        auto buf_size = buffer_size(item->buffer);

        EASYNET_EXPAND_EXPR(std::cout << "socket::start_read_some: this=" << this << ", op=" << task_name(item->task) << ", size=" << buf_size << ", addr=" << item << std::endl;)

        m_sock.async_read_some(
             boost::asio::buffer(buf_data, buf_size)
            ,[this, item]
             (const error_code &ec, std::size_t rd)
             { read_handler(item, ec, rd); }
        );
    }
    void read_handler(task_item *item, const error_code& ec, std::size_t rd) {
        auto holder = std::move(item->holder);
        auto buffer = std::move(item->buffer);
        auto funct  = std::move(item->handler);

        EASYNET_EXPAND_EXPR(
            auto size = buffer.size;
            auto task = task_name(item->task);
            std::cout << "socket::read_handler enter: this=" << this << ", op=" << task << ", size=" << size << ", addr=" << item << std::endl;
        )

        m_read_queue.pop_front_and_dispose([](task_item *item){ delete item; });

        EASYNET_TRY_CATCH_BLOCK(funct(ec, std::move(buffer), rd, std::move(holder));)

        if ( !m_read_queue.empty() && !m_read_queue.front().in_process ) {
            task_item *item2 = &(m_read_queue.front());
            start_task(item2);
        }

        EASYNET_EXPAND_EXPR(std::cout << "socket::read_handler leave: this=" << this << ", op=" << task << ", size=" << size << ", addr=" << item << std::endl;)
    }

    boost::asio::ip::tcp::socket m_sock;
    using allocator_type = handler_allocator<128>;
    queue_type m_read_queue;
    allocator_type m_read_handler_allocator;
    queue_type m_write_queue;
    allocator_type m_write_handler_allocator;
};

/***************************************************************************/

socket::socket(socket &&r)
    :pimpl{std::move(r.pimpl)}
{}

socket& socket::operator=(socket &&r) {
    pimpl = std::move(r.pimpl);
    return *this;
}

socket::socket(boost::asio::io_context& ios)
    :pimpl{std::make_unique<impl>(ios)}
{}

socket::~socket()
{}

void* socket::get_impl_details() { return &(pimpl->m_sock); }

/***************************************************************************/

boost::asio::io_context& socket::get_io_context() { return pimpl->m_sock.get_io_context(); }

void socket::connect(const char *ip, boost::uint16_t port) { return pimpl->connect(ip, port); }
void socket::connect(const char *ip, boost::uint16_t port, error_code &ec) { return pimpl->connect(ip, port, ec); }

void socket::async_connect(const char *ip, std::uint16_t port, std::function<void(const error_code &ec, impl_holder)> cb, impl_holder holder)
{ return pimpl->async_connect(ip, port, std::move(cb), std::move(holder)); }

void socket::disconnect() { return pimpl->disconnect(); }
void socket::disconnect(error_code &ec) { return pimpl->disconnect(ec); }

void socket::cancel() { return pimpl->cancel(); }
void socket::cancel(error_code &ec) { return pimpl->cancel(ec); }

void socket::close() { return pimpl->close(); }
void socket::close(error_code &ec) { return pimpl->close(ec); }

void socket::reset() { return pimpl->reset(); }
void socket::reset(error_code &ec) { return pimpl->reset(ec); }

bool socket::is_open() const { return pimpl->is_open(); }

endpoint socket::local_endpoint() const { return pimpl->local_endpoint(); }
endpoint socket::local_endpoint(error_code &ec) const { return pimpl->local_endpoint(ec); }
endpoint socket::remote_endpoint() const { return pimpl->remote_endpoint(); }
endpoint socket::remote_endpoint(error_code &ec) const { return pimpl->remote_endpoint(ec); }

std::size_t socket::available() const { return pimpl->available(); }
std::size_t socket::available(error_code &ec) const { return pimpl->available(ec); }

std::size_t socket::read_queue_size() const { return pimpl->read_queue_size(); }
std::size_t socket::write_queue_size() const { return pimpl->write_queue_size(); }

void socket::clear_read_queue() { return pimpl->clear_read_queue(); }
void socket::clear_write_queue() { return pimpl->clear_write_queue(); }

void socket::read_queue_pop_front() { return pimpl->read_queue_pop_front(1); }
void socket::read_queue_pop_front(std::size_t n) { return pimpl->read_queue_pop_front(n); }
void socket::read_queue_pop_back() { return pimpl->read_queue_pop_back(1); }
void socket::read_queue_pop_back(std::size_t n) { return pimpl->read_queue_pop_back(n); }

void socket::write_queue_pop_front() { return pimpl->write_queue_pop_front(1); }
void socket::write_queue_pop_front(std::size_t n) { return pimpl->write_queue_pop_front(n); }
void socket::write_queue_pop_back() { return pimpl->write_queue_pop_back(1); }
void socket::write_queue_pop_back(std::size_t n) { return pimpl->write_queue_pop_back(n); }

void socket::wait_write() { return pimpl->wait_write(); }
void socket::wait_write(easynet::error_code &ec) { return pimpl->wait_write(ec); }

std::size_t socket::write(const void* ptr, std::size_t size) { return pimpl->write(ptr, size); }
std::size_t socket::write(const void* ptr, std::size_t size, error_code &ec) { return pimpl->write(ptr, size, ec); }
std::size_t socket::write(const shared_buffer &buf) { return pimpl->write(buf); }
std::size_t socket::write(const shared_buffer &buf, error_code &ec) { return pimpl->write(buf, ec); }

std::size_t socket::write_some(const void* ptr, std::size_t size) { return pimpl->write_some(ptr, size); }
std::size_t socket::write_some(const void* ptr, std::size_t size, error_code &ec) { return pimpl->write_some(ptr, size, ec); }
std::size_t socket::write_some(const shared_buffer &buf) { return pimpl->write_some(buf); }
std::size_t socket::write_some(const shared_buffer &buf, error_code &ec) { return pimpl->write_some(buf, ec); }

void socket::wait_read() { return pimpl->wait_read(); }
void socket::wait_read(easynet::error_code &ec) { return pimpl->wait_read(ec); }

std::size_t socket::read(void* ptr, std::size_t size) { return pimpl->read(ptr, size); }
std::size_t socket::read(void* ptr, std::size_t size, error_code &ec) { return pimpl->read(ptr, size, ec); }
shared_buffer socket::read(std::size_t size) { return pimpl->read(size); }
shared_buffer socket::read(std::size_t size, error_code &ec) { return pimpl->read(size, ec); }

std::size_t socket::read_some(void* ptr, std::size_t size) { return pimpl->read_some(ptr, size); }
std::size_t socket::read_some(void* ptr, std::size_t size, error_code &ec) { return pimpl->read_some(ptr, size, ec); }
shared_buffer socket::read_some(std::size_t size) { return pimpl->read_some(size); }
shared_buffer socket::read_some(std::size_t size, error_code &ec) { return pimpl->read_some(size, ec); }

void socket::append_task(socket::e_task task, shared_buffer buf, handler_type cb, impl_holder holder)
{ return pimpl->append_task(task, std::move(buf), std::move(cb), std::move(holder)); }
void socket::append_task(socket::e_task task, handler_type2 cb, impl_holder holder)
{ return pimpl->append_task(task, std::move(cb), std::move(holder)); }

/***************************************************************************/

} // ns easynet
