
// Copyright (c) 2013-2022 niXman (github dotty nixman doggy pm dotty me)
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

#ifndef __easynet__shared_buffer_hpp
#define __easynet__shared_buffer_hpp

#include <memory>
#include <vector>
#include <string>
#include <cstring>
#include <cassert>

namespace easynet {

/***************************************************************************/

struct shared_buffer {
    std::shared_ptr<char> data;
    std::size_t size;
    std::size_t capacity;
    std::size_t offset;
};

/***************************************************************************/

constexpr inline std::size_t calc_capacity(std::size_t size) {
    constexpr double sqrt_of_5 = 2.2360679775;
    return static_cast<std::size_t>(size * sqrt_of_5 / 1.5);
}

inline shared_buffer buffer_alloc(std::size_t size) {
    if ( !size ) { return {}; }

    auto capacity = calc_capacity(size);
    return {
         {new char[capacity], std::default_delete<char[]>()}
        ,size
        ,capacity // capacity
        ,0u // offset
    };
}

inline shared_buffer buffer_alloc(std::size_t size, std::size_t cap) {
    if ( !size && !cap ) { return {}; }
    assert(cap >= size);

    return {
         {new char[cap], std::default_delete<char[]>()}
        ,size
        ,cap // capacity
        ,0u  // offset
    };
}

inline shared_buffer buffer_clone(const shared_buffer &buffer) {
    if ( !buffer.data ) { return {}; }

    std::size_t capacity = (std::max)(buffer.capacity, buffer.size);
    shared_buffer result = {
         {new char[capacity], std::default_delete<char[]>()}
        ,buffer.size
        ,capacity // capacity
        ,0u // offset
    };
    std::memcpy(result.data.get(), buffer.data.get(), buffer.size);

    return result;
}

inline shared_buffer buffer_clone(const shared_buffer &buffer, std::size_t size) {
    assert(size <= buffer.size);

    std::size_t capacity = (std::max)(buffer.capacity, buffer.size);
    shared_buffer result = {
         {new char[capacity], std::default_delete<char[]>()}
        ,size
        ,capacity // capacity
        ,0u // offset
    };
    std::memcpy(result.data.get(), buffer.data.get(), size);

    return result;
}

inline shared_buffer buffer_clone(const shared_buffer &buffer, std::size_t from, std::size_t len) {
    assert(len <= buffer.size && from <= buffer.size && from+len <= buffer.size);

    std::size_t capacity = (std::max)(buffer.capacity, buffer.size);
    shared_buffer result = {
         {new char[capacity], std::default_delete<char[]>()}
        ,len
        ,capacity // capacity
        ,0u // offset
    };
    std::memcpy(result.data.get(), buffer.data.get()+from, len);

    return result;
}

inline shared_buffer buffer_clone(const void *ptr, size_t size) {
    assert(ptr && size);

    shared_buffer result = {
         {new char[size], std::default_delete<char[]>()}
        ,size
        ,size // capacity
        ,0u // offset
    };
    std::memcpy(result.data.get(), ptr, size);

    return result;
}

inline void buffer_reserve(shared_buffer &buffer, std::size_t cap) {
    if ( !buffer.data ) {
        buffer = buffer_alloc(0, cap);

        return;
    }

    if ( cap <= buffer.capacity ) {
        return;
    }

    auto new_buffer = buffer_alloc(buffer.size, cap);
    std::memcpy(new_buffer.data.get(), buffer.data.get(), buffer.size);
}

inline void buffer_resize(shared_buffer &buffer, std::size_t size) {
    if ( buffer.size == 0 ) {
        buffer = buffer_alloc(size);

        return;
    }

    if ( buffer.size == size ) {
        return;
    }

    // size <= buffer.capacity
    if ( size <= buffer.capacity ) {
        shared_buffer result = {
             {buffer.data, buffer.data.get()}
            ,size
            ,buffer.capacity // capacity
            ,0u // offset
        };

        buffer = result;

        return;
    }

    // size > buffer.capacity
    auto capacity = calc_capacity(size);
    shared_buffer result = {
         {new char[capacity], std::default_delete<char[]>()}
        ,size
        ,capacity // capacity
        ,0u // offset
    };
    std::memcpy(result.data.get(), buffer.data.get(), buffer.size);

    buffer = result;
}

inline shared_buffer buffer_lshift(const shared_buffer &buffer, std::size_t bytes) {
    assert(bytes <= buffer.size);

    shared_buffer result = {
        {buffer.data, buffer.data.get()+bytes}
        ,buffer.size-bytes
        ,buffer.capacity-bytes // capacity
        ,buffer.offset+bytes // offset
    };

    return result;
}

/***************************************************************************/

inline std::size_t buffer_size(const shared_buffer &buffer) {
    return buffer.size;
}

inline std::size_t buffer_capacity(const shared_buffer &buffer) {
    return buffer.capacity;
}

inline const char* buffer_data(const shared_buffer &buffer) {
    return buffer.data.get();
}

inline char* buffer_data(shared_buffer &buffer) {
    return buffer.data.get();
}

inline const char* buffer_data_unshifted(const shared_buffer &buffer) {
    return buffer.data.get() - buffer.offset;
}

inline char* buffer_data_unshifted(shared_buffer &buffer) {
    return buffer.data.get() - buffer.offset;
}

inline std::size_t buffer_refs(const shared_buffer &buffer) {
    return static_cast<std::size_t>(buffer.data.use_count());
}

/***************************************************************************/

template<
     typename ByteT
    ,typename = typename std::enable_if<std::is_fundamental<ByteT>::value && sizeof(ByteT) == 1>::type
>
inline shared_buffer make_nonowning_buffer(const std::vector<ByteT> &vec) {
    shared_buffer buf{
         {const_cast<char *>(static_cast<const char *>(vec.data())), [](char*){}}
        ,vec.size()
        ,vec.size()
        ,0u
    };

    return buf;
}

inline shared_buffer make_nonowning_buffer(const std::string &str) {
    shared_buffer buf{
         {const_cast<char *>(static_cast<const char *>(str.data())), [](char*){}}
        ,str.size()
        ,str.size()
        ,0u
    };

    return buf;
}

/***************************************************************************/

} // namespace easynet

#endif // __easynet__shared_buffer_hpp
