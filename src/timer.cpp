
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

#include <easynet/timer.hpp>

#include <boost/asio/steady_timer.hpp>

namespace easynet {

/***************************************************************************/

struct timer::impl {
    impl(boost::asio::io_context &ios)
        :m_timer{ios}
        ,m_started{false}
    {}
    ~impl()
    {}

    void start(std::size_t ms, timeout_handler cb) {
        if ( m_started ) {
            cb(boost::asio::error::already_started);

            return;
        }

        m_started = true;

        m_timer.expires_from_now(std::chrono::milliseconds(ms));
        m_timer.async_wait(
            [this, cb=std::move(cb)]
            (const error_code &ec) mutable {
                m_started = false;
                cb(ec);
            }
        );
    }
    void stop() {
        m_started = false;
        m_timer.cancel();
    }
    bool started() const { return m_started; }

    boost::asio::steady_timer m_timer;
    bool m_started;
};

/***************************************************************************/

timer::timer(boost::asio::io_context &ios)
    :pimpl{std::make_shared<impl>(ios)}
{}

/***************************************************************************/

void timer::start(std::size_t ms, timeout_handler cb) { return pimpl->start(ms, std::move(cb)); }
void timer::stop() { return pimpl->stop(); }
bool timer::started() const { return pimpl->started(); }


/***************************************************************************/

} // ns easynet
