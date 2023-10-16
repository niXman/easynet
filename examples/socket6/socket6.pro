
TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

#DEFINES += \
#    BOOST_ASIO_NO_DYNAMIC_BUFFER_V1

QMAKE_CXXFLAGS += \
    -std=c++14 \
    -pthread

QMAKE_LFLAGS += \
    -pthread

INCLUDEPATH += \
    ../../include

SOURCES += \
    socket6.cpp \
    \
    ../../src/acceptor.cpp \
    ../../src/socket.cpp \
    ../../src/timer.cpp

HEADERS += \
    ../../include/easynet/acceptor.hpp \
    ../../include/easynet/handler_allocator.hpp \
    ../../include/easynet/preallocated_handler.hpp \
    ../../include/easynet/preallocated_handler_invoker.hpp \
    ../../include/easynet/shared_buffer.hpp \
    ../../include/easynet/socket.hpp
