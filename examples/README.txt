
- acceptor1: the simplest example usign acceptor inside a loop with synchronous accept.
- acceptor2: another very simple example usign acceptor inside a loop with synchronous accept-socket-factory.
- acceptor3: the example of using async-accept and free function as connection handler.
- acceptor4: refactored `acceptor3` example using a class and member-function as connection handler.
- acceptor5: refactored `acceptor4` example with additional `session` class for accepted connection.

- socket1  : just a test for read/write queue pop_front/pop_back.
- socket2  : another client implementation wrapped as class.
- socket3  : just a example using read/write handlers as member-functions.
- socket4  : the example uses std::enable_shared_from_this<> for class client.
- socket5  : just a demonstration of using async_read_some/async_write_some.
- socket6  : just a demonstration of using async_read_until().

- timer1   : the example of using periodic timer class inherited std::enable_shared_from_this<>
