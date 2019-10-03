[![Build Status](https://travis-ci.org/niXman/yas.svg?branch=master)](https://travis-ci.org/niXman/easynet)

easynet
=======

The wrappers on top of boost.asio

examples
=======
1. [acceptor1](https://github.com/niXman/easynet/blob/master/examples/acceptor1/acceptor1.cpp) - synchronous echo server
2. [acceptor2](https://github.com/niXman/easynet/blob/master/examples/acceptor2/acceptor2.cpp) - synchronous echo server
3. [acceptor3](https://github.com/niXman/easynet/blob/master/examples/acceptor3/acceptor3.cpp) - asynchronous echo server with async accept & accept handler
4. [acceptor4](https://github.com/niXman/easynet/blob/master/examples/acceptor4/acceptor4.cpp) - asynchronous echo server class with async accept & accept handler as member-function
5. [acceptor5](https://github.com/niXman/easynet/blob/master/examples/acceptor5/acceptor5.cpp) - asynchronous echo server class with async accept & session class
6. [socket1](https://github.com/niXman/easynet/blob/master/examples/socket1/socket1.cpp) - asynchronous echo client with read/write handlers and test for socket's read/write queue
7. [socket2](https://github.com/niXman/easynet/blob/master/examples/socket2/socket2.cpp) - asynchronous echo client class with read/write handlers as member-fucntion
8. [socket3](https://github.com/niXman/easynet/blob/master/examples/socket3/socket3.cpp) - the same as above with minor diffs
9. [socket4](https://github.com/niXman/easynet/blob/master/examples/socket4/socket4.cpp) - the same as above with minor diffs
10. [socket5](https://github.com/niXman/easynet/blob/master/examples/socket5/socket5.cpp) - asynchronous echo client class with read_some/write_some handlers as member-fucntion
10. [socket6](https://github.com/niXman/easynet/blob/master/examples/socket6/socket6.cpp) - asynchronous echo client class that uses `std::enable_shared_from_this<>` by inheritance with read_some/write_some handlers as member-fucntion
10. [timer1](https://github.com/niXman/easynet/blob/master/examples/timer1/timer1.cpp) - timer class that uses `std::enable_shared_from_this<>` by inheritance
