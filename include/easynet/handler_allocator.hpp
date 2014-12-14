
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

#ifndef _easynet__handler_allocator_hpp
#define _easynet__handler_allocator_hpp

#include <cstddef>
#include <boost/noncopyable.hpp>
#include <boost/aligned_storage.hpp>

namespace easynet {

/***************************************************************************/

template<size_t alloc_size>
struct handler_allocator :private boost::noncopyable {
	handler_allocator()
		:storage()
		,in_use(false)
	{}

	~handler_allocator() {}

	void* allocate(size_t size) {
		if ( !in_use && (size <= storage.size) ) {
			in_use = true;
			return storage.address();
		}
		return ::operator new(size);
	}

	void deallocate(void* pointer) {
		if ( storage.address() == pointer ) {
			in_use = false;
			return;
		}
		::operator delete(pointer);
	}

private:
	boost::aligned_storage<alloc_size> storage;
	bool in_use;
};

/***************************************************************************/

} // namespace easynet

#endif // _easynet__handler_allocator_hpp
