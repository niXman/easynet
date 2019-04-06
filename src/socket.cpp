
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
#include <easynet/preallocated_handler_invoker.hpp>
#include <easynet/preallocated_handler.hpp>
#include <easynet/socket.hpp>

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>

#include <deque>

namespace easynet {

/***************************************************************************/

struct socket::impl {
    impl(boost::asio::io_context& ios)
        :m_sock(ios)
        ,m_read_in_process(false)
        ,m_read_queue()
        ,m_write_in_process(false)
        ,m_write_queue()
    {}
    ~impl() {
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

    void async_connect(const char *ip, std::uint16_t port, std::function<void(const error_code &ec)> cb) {
        m_sock.async_connect(
             boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(ip), port)
            ,[cb=std::move(cb)]
             (const error_code &ec)
             { cb(ec); }
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
        clear_read_queue();
        clear_write_queue();
        cancel();
    }
    void reset(error_code& ec) {
        clear_read_queue();
        clear_write_queue();
        cancel(ec);
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

    void clear_read_queue() { m_read_queue.clear(); }
    void clear_write_queue() { m_write_queue.clear(); }

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
        shared_buffer buffer;
        handler_type handler;
    };

    /**  */
    void start_write() {
        m_write_in_process = true;
        task_item item = std::move(m_write_queue.front());
        m_write_queue.pop_front();

        auto buf_data = buffer_data(item.buffer);
        auto buf_size = buffer_size(item.buffer);
        boost::asio::async_write(
             m_sock
            ,boost::asio::buffer(buf_data, buf_size)
            ,make_preallocated_handler(
                 m_write_handler_allocator
                ,[this, item=std::move(item)]
                 (const error_code& ec, std::size_t wr) mutable
                 { write_handler(std::move(item), ec, wr); }
            )
        );
    }
    void write_handler(task_item item, const error_code& ec, std::size_t wr) {
        item.handler(ec, std::move(item.buffer), wr);

        if ( !m_write_queue.empty() ) {
            start_write();
        } else {
            m_write_in_process = false;
        }
    }

    /**  */
    void start_write_some() {
        m_write_in_process = true;
        task_item item = std::move(m_write_queue.front());
        m_write_queue.pop_front();

        auto buf_data = buffer_data(item.buffer);
        auto buf_size = buffer_size(item.buffer);
        m_sock.async_write_some(
             boost::asio::buffer(buf_data, buf_size)
            ,make_preallocated_handler(
                 m_write_handler_allocator
                ,[this, item=std::move(item)]
                 (const error_code& ec, std::size_t wr) mutable
                 { write_some_handler(std::move(item), ec, wr); }
            )
        );
    }
    void write_some_handler(task_item item, const error_code& ec, std::size_t wr) {
        item.handler(ec, std::move(item.buffer), wr);

        if ( !m_write_queue.empty() ) {
            start_write_some();
        } else {
            m_write_in_process = false;
        }
    }

    /**  */
    void start_read() {
        m_read_in_process = true;
        task_item item = std::move(m_read_queue.front());
        m_read_queue.pop_front();

        if ( !item.buffer.data ) {
            item.buffer = buffer_alloc(item.buffer.size);
        }

        auto buf_data = buffer_data(item.buffer);
        auto buf_size = buffer_size(item.buffer);
        boost::asio::async_read(
             m_sock
            ,boost::asio::buffer(buf_data, buf_size)
            ,make_preallocated_handler(
                 m_read_handler_allocator
                ,[this, item=std::move(item)]
                 (const error_code& ec, std::size_t rd) mutable
                 { read_handler(std::move(item), ec, rd); }
            )
        );
    }
    void read_handler(task_item item, const error_code& ec, std::size_t rd) {
        item.handler(ec, std::move(item.buffer), rd);

        if ( !m_read_queue.empty() ) {
            start_read();
        } else {
            m_read_in_process = false;
        }
    }

    /**  */
    void start_read_some() {
        m_read_in_process = true;
        task_item item = std::move(m_read_queue.front());
        m_read_queue.pop_front();

        if ( !item.buffer.data ) {
            item.buffer = buffer_alloc(item.buffer.size);
        }

        auto buf_data = buffer_data(item.buffer);
        auto buf_size = buffer_size(item.buffer);
        m_sock.async_read_some(
             boost::asio::buffer(buf_data, buf_size)
            ,make_preallocated_handler(
                 m_read_handler_allocator
                ,[this, item=std::move(item)]
                 (const error_code& ec, std::size_t rd) mutable
                 { read_some_handler(std::move(item), ec, rd); }
            )
        );
    }
    void read_some_handler(task_item item, const error_code& ec, std::size_t rd) {
        item.handler(ec, std::move(item.buffer), rd);

        if ( !m_read_queue.empty() ) {
            start_read_some();
        } else {
            m_read_in_process = false;
        }
    }

    void append_write_task(shared_buffer buf, handler_type cb) {
        task_item item{std::move(buf), std::move(cb)};
        m_write_queue.push_back(std::move(item));
        if ( !m_write_in_process ) { start_write(); }
    }
    void append_write_some_task(shared_buffer buf, handler_type cb) {
        task_item item{std::move(buf), std::move(cb)};
        m_write_queue.push_back(std::move(item));
        if ( !m_write_in_process ) { start_write_some(); }
    }
    void append_read_task(shared_buffer buf, handler_type cb) {
        task_item item{std::move(buf), std::move(cb)};
        m_read_queue.push_back(std::move(item));
        if ( !m_read_in_process ) { start_read(); }
    }
    void append_read_some_task(shared_buffer buf, handler_type cb) {
        task_item item{std::move(buf), std::move(cb)};
        m_read_queue.push_back(std::move(item));
        if ( !m_read_in_process ) { start_read_some(); }
    }

    ///////////////////////////////////////////////////////////////////////////
    boost::asio::ip::tcp::socket m_sock;

    using allocator_type = handler_allocator<128>;
    bool m_read_in_process;
    std::deque<task_item> m_read_queue;
    allocator_type m_read_handler_allocator;
    bool m_write_in_process;
    std::deque<task_item> m_write_queue;
    allocator_type m_write_handler_allocator;
};

/***************************************************************************/

socket::socket(boost::asio::io_context& ios)
    :pimpl{std::make_shared<impl>(ios)}
{}

void* socket::get_impl_details() { return &(pimpl->m_sock); }

/***************************************************************************/

boost::asio::io_context& socket::get_io_context() { return pimpl->m_sock.get_io_context(); }

void socket::connect(const char *ip, boost::uint16_t port) { return pimpl->connect(ip, port); }
void socket::connect(const char *ip, boost::uint16_t port, error_code &ec) { return pimpl->connect(ip, port, ec); }

void socket::async_connect(const char *ip, std::uint16_t port, std::function<void(const error_code &ec)> cb) { return pimpl->async_connect(ip, port, std::move(cb)); }

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

std::size_t socket::write(const void* ptr, std::size_t size) { return pimpl->write(ptr, size); }
std::size_t socket::write(const void* ptr, std::size_t size, error_code &ec) { return pimpl->write(ptr, size, ec); }
std::size_t socket::write(const shared_buffer &buf) { return pimpl->write(buf); }
std::size_t socket::write(const shared_buffer &buf, error_code &ec) { return pimpl->write(buf, ec); }

std::size_t socket::write_some(const void* ptr, std::size_t size) { return pimpl->write_some(ptr, size); }
std::size_t socket::write_some(const void* ptr, std::size_t size, error_code &ec) { return pimpl->write_some(ptr, size, ec); }
std::size_t socket::write_some(const shared_buffer &buf) { return pimpl->write_some(buf); }
std::size_t socket::write_some(const shared_buffer &buf, error_code &ec) { return pimpl->write_some(buf, ec); }

std::size_t socket::read(void* ptr, std::size_t size) { return pimpl->read(ptr, size); }
std::size_t socket::read(void* ptr, std::size_t size, error_code &ec) { return pimpl->read(ptr, size, ec); }
shared_buffer socket::read(std::size_t size) { return pimpl->read(size); }
shared_buffer socket::read(std::size_t size, error_code &ec) { return pimpl->read(size, ec); }

std::size_t socket::read_some(void* ptr, std::size_t size) { return pimpl->read_some(ptr, size); }
std::size_t socket::read_some(void* ptr, std::size_t size, error_code &ec) { return pimpl->read_some(ptr, size, ec); }
shared_buffer socket::read_some(std::size_t size) { return pimpl->read_some(size); }
shared_buffer socket::read_some(std::size_t size, error_code &ec) { return pimpl->read_some(size, ec); }

void socket::append_write_task(shared_buffer buf, handler_type cb) { return pimpl->append_write_task(std::move(buf), std::move(cb)); }
void socket::append_write_some_task(shared_buffer buf, handler_type cb) { return pimpl->append_write_some_task(std::move(buf), std::move(cb)); }
void socket::append_read_task(shared_buffer buf, handler_type cb) { return pimpl->append_read_task(std::move(buf), std::move(cb)); }
void socket::append_read_some_task(shared_buffer buf, handler_type cb) { return pimpl->append_read_some_task(std::move(buf), std::move(cb)); }

/***************************************************************************/

} // ns easynet
