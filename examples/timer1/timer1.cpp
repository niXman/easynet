
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

#include <boost/asio/io_context.hpp>

#include <iostream>

/***************************************************************************/

struct timer_object: std::enable_shared_from_this<timer_object> {
    timer_object(boost::asio::io_context &ios)
        :m_timer{ios}
    {}

    void start() {
        m_timer.start(
             1000
            ,[this, self=shared_from_this()]
             (const easynet::error_code &ec)
             { on_timeout_0(ec); }
        );
    }

    void on_timeout_0(const easynet::error_code &ec) {
        std::cout << "on_timeout_0(): ec=" << ec << std::endl;

        m_timer.start(1000, shared_from_this(), &timer_object::on_timeout_1);
    }
    void on_timeout_1(const easynet::error_code &ec) {
        std::cout << "on_timeout_1(): ec=" << ec << std::endl;

    }

    easynet::timer m_timer;
};

/***************************************************************************/

int main(int, char**) {
    try {
        boost::asio::io_context ios;

        {
            auto timer = std::make_shared<timer_object>(ios);
            timer->start();
        }

        ios.run();
    } catch (const std::exception &ex) {
        std::cout << "[exception]: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

/***************************************************************************/
