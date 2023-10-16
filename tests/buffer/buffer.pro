TEMPLATE = app
CONFIG += console c++14
CONFIG -= app_bundle
CONFIG -= qt

QMAKE_CXXFLAGS += \
    -fsanitize=address

LIBS += \
    -lasan

INCLUDEPATH += \
    ../../include

SOURCES += \
    main.cpp
