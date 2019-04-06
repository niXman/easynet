
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
    timer1.cpp \
    \
    ../../src/timer.cpp

HEADERS += \
    ../../include/easynet/handler_allocator.hpp \
    ../../include/easynet/preallocated_handler.hpp \
    ../../include/easynet/preallocated_handler_invoker.hpp \
    ../../include/easynet/timer.hpp
