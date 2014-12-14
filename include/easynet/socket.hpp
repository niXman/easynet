
// Copyright (c) 2013,2014 niXman (i dotty nixman doggy gmail dotty com)
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

#ifndef _easynet__socket_hpp
#define _easynet__socket_hpp

#include <deque>
#include <stdexcept>

#include <easynet/shared_buffer.hpp>
#include <easynet/handler_allocator.hpp>
#include <easynet/preallocated_handler_invoker.hpp>
#include <easynet/make_custom_allocated_handler.hpp>

#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>

namespace easynet {

typedef boost::asio::ip::tcp::endpoint endpoint;
typedef boost::system::error_code error_code;

/***************************************************************************/

struct socket: private boost::noncopyable {
	explicit socket(boost::asio::io_service& ios)
		:sock(ios)
		,read_in_process(false)
		,read_queue()
		,write_in_process(false)
		,write_queue()
	{}
	explicit socket(socket &&r)
		:sock(std::move(r.sock))
		,read_in_process(r.read_in_process)
		,read_queue(std::move(r.read_queue))
		,read_handler_allocator()
		,write_in_process(r.write_in_process)
		,write_queue(std::move(r.write_queue))
		,write_handler_allocator()
	{}

	virtual ~socket() {
		boost::system::error_code ec;
		disconnect(ec);
	}

	/**  */
	boost::asio::io_service& get_io_service() { return sock.get_io_service(); }

	/**  */
	boost::asio::ip::tcp::socket& get_socket() { return sock; }

	/**  */
	void connect(const std::string& ip, boost::uint16_t port) {
		sock.connect(endpoint(boost::asio::ip::address::from_string(ip), port));
	}
	void connect(const std::string& ip, boost::uint16_t port, error_code& ec) {
		sock.connect(endpoint(boost::asio::ip::address::from_string(ip), port), ec);
	}

	/**  */
	template<typename F>
	void async_connect(const std::string& ip, boost::uint16_t port, F &&handler) {
		sock.async_connect(endpoint(boost::asio::ip::address::from_string(ip), port), handler);
	}

	/**  */
	void disconnect() {
		reset();
		if ( is_open() ) {
			sock.shutdown(boost::asio::ip::tcp::socket::shutdown_both);
			sock.close();
		}
	}
	void disconnect(error_code& ec) {
		reset(ec);
		if ( ec ) return;
		if ( is_open() ) {
			sock.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
			if ( ec ) return;
			sock.close(ec);
		}
	}

	/**  */
	void cancel() { sock.cancel(); }
	void cancel(error_code& ec) { sock.cancel(ec); }

	/**  */
	void close() { sock.close(); }
	void close(error_code& ec) { sock.close(ec); }

	/**  */
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

	/**  */
	bool is_open() const { return sock.is_open(); }

	/**  */
	std::size_t available() const { return sock.available(); }
	std::size_t available(error_code& ec) const { return sock.available(ec); }

	/**  */
	std::size_t read_queue_size() const { return read_queue.size(); }
	std::size_t write_queue_size() const { return write_queue.size(); }

	/**  */
	void clear_read_queue() { read_queue.clear(); }
	void clear_write_queue() { write_queue.clear(); }

	/**  */
	std::size_t write(const void* ptr, size_t size) {
		return boost::asio::write(sock, boost::asio::buffer(ptr, size));
	}
	std::size_t write(const void* ptr, size_t size, error_code& ec) {
		return boost::asio::write(sock, boost::asio::buffer(ptr, size), ec);
	}

	/**  */
	template<typename F>
	void async_write(shared_buffer buf, F &&handler) {
		if ( !check_args(sock, buf, handler) ) return;
		/**  */
		queue_item_ptr item = std::make_shared<queue_item_ptr::element_type>();
		item->handler = std::move(handler);
		item->buffer = std::move(buf);

		/**  */
		write_queue.push_back(std::move(item));
		if ( !write_in_process ) {
			write_exec();
		}
	}
	template<typename Obj>
	void async_write(shared_buffer buf, Obj* o, void(Obj::*m)(const error_code&, shared_buffer, size_t)) {
		async_write(buf, [o, m](const error_code &ec, shared_buffer buf, size_t wr){ (o->*m)(ec, buf, wr); });
	}
	template<typename Obj>
	void async_write(shared_buffer buf, std::shared_ptr<Obj> o, void(Obj::*m)(const error_code&, shared_buffer, size_t)) {
		async_write(buf, [o, m](const error_code &ec, shared_buffer buf, size_t wr){ (o.get()->*m)(ec, buf, wr); });
	}

	/**  */
	template<typename F>
	void async_write_some(shared_buffer buf, F &&handler) {
		if ( !check_args(sock, buf, handler) ) return;
		/**  */
		queue_item_ptr item = std::make_shared<queue_item_ptr::element_type>();
		item->handler = std::move(handler);
		item->buffer = std::move(buf);

		/**  */
		write_queue.push_back(std::move(item));
		if ( !write_in_process ) {
			write_some_exec();
		}
	}
	template<typename Obj>
	void async_write_some(shared_buffer buf, Obj* o, void(Obj::*m)(const error_code&, shared_buffer, size_t)) {
		async_write_some(buf, [o, m](const error_code &ec, shared_buffer buf, size_t wr){ (o->*m)(ec, buf, wr); });
	}
	template<typename Obj>
	void async_write_some(shared_buffer buf, std::shared_ptr<Obj> o, void(Obj::*m)(const error_code&, shared_buffer, size_t)) {
		async_write_some(buf, [o, m](const error_code &ec, shared_buffer buf, size_t wr){ (o.get()->*m)(ec, buf, wr); });
	}

	/**  */
	size_t read(void* ptr, size_t size) {
		return boost::asio::read(sock, boost::asio::buffer(ptr, size));
	}
	size_t read(void* ptr, size_t size, error_code& ec) {
		return boost::asio::read(sock, boost::asio::buffer(ptr, size), ec);
	}

	/**  */
	template<typename F>
	void async_read(size_t size, F &&handler) {
		if ( !check_args(sock, size, handler) ) return;
		/**  */
		queue_item_ptr item = std::make_shared<queue_item_ptr::element_type>();
		item->handler = std::move(handler);
		item->buffer.size = size;

		/**  */
		read_queue.push_back(std::move(item));
		if ( !read_in_process ) {
			read_exec();
		}
	}
	template<typename F>
	void async_read(shared_buffer buf, F &&handler) {
		if ( !check_args(sock, buf, handler) ) return;
		/**  */
		queue_item_ptr item = std::make_shared<queue_item_ptr::element_type>();
		item->handler = std::move(handler);
		item->buffer = std::move(buf);

		/**  */
		read_queue.push_back(std::move(item));
		if ( !read_in_process ) {
			read_exec();
		}
	}
	template<typename Obj>
	void async_read(size_t size, Obj* o, void(Obj::*m)(const error_code&, shared_buffer, size_t)) {
		async_read(size, [o, m](const error_code &ec, shared_buffer buf, size_t rd){ (o->*m)(ec, buf, rd); });
	}
	template<typename Obj>
	void async_read(shared_buffer buf, Obj* o, void(Obj::*m)(const error_code&, shared_buffer, size_t)) {
		async_read(buf, [o, m](const error_code &ec, shared_buffer buf, size_t rd){ (o->*m)(ec, buf, rd); });
	}
	template<typename Obj>
	void async_read(size_t size, std::shared_ptr<Obj> o, void(Obj::*m)(const error_code&, shared_buffer, size_t)) {
		async_read(size, [o, m](const error_code &ec, shared_buffer buf, size_t rd){ (o.get()->*m)(ec, buf, rd); });
	}
	template<typename Obj>
	void async_read(shared_buffer buf, std::shared_ptr<Obj> o, void(Obj::*m)(const error_code&, shared_buffer, size_t)) {
		async_read(buf, [o, m](const error_code &ec, shared_buffer buf, size_t rd){ (o.get()->*m)(ec, buf, rd); });
	}

	/**  */
	template<typename F>
	void async_read_some(size_t size, F &&handler) {
		if ( !check_args(sock, size, handler) ) return;
		/**  */
		queue_item_ptr item = std::make_shared<queue_item_ptr::element_type>();
		item->handler = std::move(handler);
		item->buffer.size = size;

		/**  */
		read_queue.push_back(std::move(item));
		if ( !read_in_process ) {
			read_some_exec();
		}
	}
	template<typename F>
	void async_read_some(shared_buffer buf, F &&handler) {
		if ( !check_args(sock, buf, handler) ) return;
		/**  */
		queue_item_ptr item = std::make_shared<queue_item_ptr::element_type>();
		item->handler = std::move(handler);
		item->buffer = std::move(buf);

		/**  */
		read_queue.push_back(std::move(item));
		if ( !read_in_process ) {
			read_some_exec();
		}
	}
	template<typename Obj>
	void async_read_some(size_t size, Obj* o, void(Obj::*m)(const error_code&, shared_buffer, size_t)) {
		async_read_some(size, [o, m](const error_code &ec, shared_buffer buf, size_t rd){ (o->*m)(ec, buf, rd); });
	}
	template<typename Obj>
	void async_read_some(shared_buffer buf, Obj* o, void(Obj::*m)(const error_code&, shared_buffer, size_t)) {
		async_read_some(buf, [o, m](const error_code &ec, shared_buffer buf, size_t rd){ (o->*m)(ec, buf, rd); });
	}
	template<typename Obj>
	void async_read_some(size_t size, std::shared_ptr<Obj> o, void(Obj::*m)(const error_code&, shared_buffer, size_t)) {
		async_read_some(size, [o, m](const error_code &ec, shared_buffer buf, size_t rd){ (o.get()->*m)(ec, buf, rd); });
	}
	template<typename Obj>
	void async_read_some(shared_buffer buf, std::shared_ptr<Obj> o, void(Obj::*m)(const error_code&, shared_buffer, size_t)) {
		async_read_some(buf, [o, m](const error_code &ec, shared_buffer buf, size_t rd){ (o.get()->*m)(ec, buf, rd); });
	}

private:
	/**  */
	template<typename F>
	static bool check_args(const boost::asio::ip::tcp::socket& sock, const shared_buffer& buf, const F& handler) {
		if ( !sock.is_open() ) {
			handler(error_code(boost::asio::error::not_connected), buf, 0);
			return false;
		}
		if ( !buffer_data(buf) || !buffer_size(buf) ) {
			handler(error_code(boost::asio::error::invalid_argument), buf, 0);
			return false;
		}

		return true;
	}
	template<typename F>
	static bool check_args(const boost::asio::ip::tcp::socket& sock, std::size_t size, const F& handler) {
		if ( !sock.is_open() ) {
			handler(error_code(boost::asio::error::not_connected), shared_buffer(), 0);
			return false;
		}
		if ( !size ) {
			handler(error_code(boost::asio::error::invalid_argument), shared_buffer(), 0);
			return false;
		}

		return true;
	}

private:
	/**  */
	typedef std::function<void(const error_code&, shared_buffer, size_t)> handler_type;

	/**  */
	struct queue_item {
		handler_type handler;
		shared_buffer buffer;
	};
	using queue_item_ptr = std::shared_ptr<queue_item>;

	/**  */
	void write_exec() {
		write_in_process = true;
		queue_item_ptr item = write_queue.front();
		write_queue.pop_front();

		boost::asio::async_write(
			 sock
			,boost::asio::buffer(buffer_data(item->buffer), buffer_size(item->buffer))
			,make_custom_preallocated_handler(
				 write_handler_allocator
				,[this, item](const error_code& ec, size_t wr) {
						write_handler(std::move(item), ec, wr);
				 }
			)
		);
	}
	void write_handler(queue_item_ptr item, const error_code& ec, size_t wr) {
		item->handler(ec, std::move(item->buffer), wr);

		if ( !write_queue.empty() ) {
			write_exec();
		} else {
			write_in_process = false;
		}
	}

	/**  */
	void write_some_exec() {
		queue_item_ptr item = write_queue.front();
		write_queue.pop_front();
		write_in_process = true;

		sock.async_write_some(
			 boost::asio::buffer(buffer_data(item->buffer), buffer_size(item->buffer))
			,make_custom_preallocated_handler(
				 write_handler_allocator
				,[this, item](const error_code& ec, size_t wr) {
						write_some_handler(std::move(item), ec, wr);
				 }
			)
		);
	}
	void write_some_handler(queue_item_ptr item, const error_code& ec, size_t wr) {
		item->handler(ec, std::move(item->buffer), wr);

		if ( !write_queue.empty() ) {
			write_some_exec();
		} else {
			write_in_process = false;
		}
	}

	/**  */
	void read_exec() {
		queue_item_ptr item = read_queue.front();
		read_queue.pop_front();
		read_in_process = true;

		if ( !item->buffer.data ) {
			item->buffer = buffer_alloc(item->buffer.size);
		}

		boost::asio::async_read(
			 sock
			,boost::asio::buffer(buffer_data(item->buffer), buffer_size(item->buffer))
			,make_custom_preallocated_handler(
				 read_handler_allocator
				,[this, item](const error_code& ec, size_t rd) {
						read_handler(std::move(item), ec, rd);
				 }
			)
		);
	}
	void read_handler(queue_item_ptr item, const error_code& ec, size_t rd) {
		item->handler(ec, std::move(item->buffer), rd);

		if ( !read_queue.empty() ) {
			read_exec();
		} else {
			read_in_process = false;
		}
	}

	/**  */
	void read_some_exec() {
		queue_item_ptr item = read_queue.front();
		read_queue.pop_front();
		read_in_process = true;

		if ( !item->buffer.data ) {
			item->buffer = buffer_alloc(item->buffer.size);
		}

		sock.async_read_some(
			 boost::asio::buffer(buffer_data(item->buffer), buffer_size(item->buffer))
			,make_custom_preallocated_handler(
				 read_handler_allocator
				,[this, item](const error_code& ec, size_t rd) {
						read_some_handler(std::move(item), ec, rd);
				 }
			)
		);
	}
	void read_some_handler(queue_item_ptr item, const error_code& ec, size_t rd) {
		item->handler(ec, std::move(item->buffer), rd);

		if ( !read_queue.empty() ) {
			read_some_exec();
		} else {
			read_in_process = false;
		}
	}

private:
	typedef handler_allocator<512> allocator_type;

	boost::asio::ip::tcp::socket sock;

	bool read_in_process;
	std::deque<queue_item_ptr> read_queue;
	allocator_type read_handler_allocator;

	bool write_in_process;
	std::deque<queue_item_ptr> write_queue;
	allocator_type write_handler_allocator;
};

/***************************************************************************/

typedef std::shared_ptr<socket> socket_ptr;

/***************************************************************************/

} // namespace easynet

#endif // _easynet__socket_hpp
