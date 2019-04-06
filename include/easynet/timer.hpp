
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

#ifndef __easynet__timer_hpp
#define __easynet__timer_hpp

#include <easynet/typedefs.hpp>

#include <memory>
#include <functional>

namespace easynet {

/***************************************************************************/

struct timer {
    timer(const timer &) = delete;
    timer& operator= (const timer &) = delete;
    timer(timer &&) = default;
    timer& operator= (timer &&) = default;

    timer(boost::asio::io_context &ios);

    using timeout_handler = std::function<void(const error_code &ec)>;
    void start(std::size_t ms, timeout_handler cb);
    template<typename Obj>
    void start(std::size_t ms, Obj* o, void(Obj::*m)(const error_code &ec)) {
        start(
             ms
            ,[this, o, m]
             (const error_code &ec)
             { (o->*m)(ec); }
        );
    }
    template<typename Obj>
    void start(std::size_t ms, std::shared_ptr<Obj> o, void(Obj::*m)(const error_code &ec)) {
        start(
             ms
            ,[this, o=std::move(o), m]
             (const error_code &ec)
             { (o.get()->*m)(ec); }
        );
    }

    void stop();
    bool started() const;

private:
    struct impl;
    std::shared_ptr<impl> pimpl;
};

/***************************************************************************/

} // ns easynet

#endif // __easynet__timer_hpp
