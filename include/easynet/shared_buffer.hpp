
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

#ifndef _easynet__shared_buffer_hpp
#define _easynet__shared_buffer_hpp

#include <memory>
#include <cstring>
#include <cassert>

namespace easynet {

/***************************************************************************/

struct shared_buffer {
	std::shared_ptr<char> data;
	size_t size;
};

/***************************************************************************/

inline shared_buffer buffer_alloc(size_t size) {
	if ( !size ) return shared_buffer();
	return { {new char[size], std::default_delete<char[]>()}, size };
}

inline shared_buffer buffer_clone(const shared_buffer& buffer) {
	if ( !buffer.size ) return shared_buffer();
	shared_buffer result = { {new char[buffer.size], std::default_delete<char[]>()}, buffer.size };
	std::memcpy(result.data.get(), buffer.data.get(), buffer.size);
	return result;
}

inline shared_buffer buffer_clone(const shared_buffer& buffer, size_t size) {
	assert(size <= buffer.size);
	shared_buffer result = { {new char[size], std::default_delete<char[]>()}, size };
	std::memcpy(result.data.get(), buffer.data.get(), size);
	return result;
}

inline shared_buffer buffer_clone(const shared_buffer& buffer, size_t from, size_t size) {
	assert(size > 0 && size <= buffer.size && from <= buffer.size && from+size <= buffer.size);
	shared_buffer result = { {new char[size], std::default_delete<char[]>()}, size };
	std::memcpy(result.data.get(), buffer.data.get()+from, size);
	return result;
}

inline shared_buffer buffer_clone(const void* ptr, size_t size) {
	assert(ptr && size);
	shared_buffer result = { {new char[size], std::default_delete<char[]>()}, size };
	std::memcpy(result.data.get(), ptr, size);
	return result;
}

inline shared_buffer buffer_shift(const shared_buffer& buffer, size_t bytes) {
	shared_buffer result = {
		{buffer.data, buffer.data.get()+bytes}
		,buffer.size-bytes
	};
	return result;
}

/***************************************************************************/

inline size_t buffer_size(const shared_buffer& buffer) {
	return buffer.size;
}

inline char* buffer_data(const shared_buffer& buffer) {
	return (char*)buffer.data.get();
}

inline size_t buffer_refs(const shared_buffer& buffer) {
	return buffer.data.use_count();
}

/***************************************************************************/

} // namespace easynet

#endif // _easynet__shared_buffer_hpp
