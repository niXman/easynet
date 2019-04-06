
TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

QMAKE_CXXFLAGS += \
    -std=c++14

INCLUDEPATH += \
    ../../include

LIBS += \
    -L/usr/local/lib \
    -lboost_system \
    -pthread

SOURCES += \
    socket1.cpp \
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
