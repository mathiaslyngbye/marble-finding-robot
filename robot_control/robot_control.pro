TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    FuzzyDefault.cpp \
    localization.cpp \
    marbleDetect.cpp

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv

HEADERS += \
    laserscanner.h \
    FuzzyDefault.h \
    localization.h \
    marbleDetect.h

# Library: FuzzyLite
INCLUDEPATH += $$PWD/../../marble-finding-robot/libraries/fuzzylite-6.0/fuzzylite
debug
{
    LIBS += -L$$PWD/../../marble-finding-robot/libraries/fuzzylite-6.0/fuzzylite/debug/bin/ -lfuzzylite-debug
}

release
{
    LIBS += -L$$PWD/../../marble-finding-robot/libraries/fuzzylite-6.0/fuzzylite/release/bin/ -lfuzzylite
}

DISTFILES += \
    fuzzybugcontroller.fll


