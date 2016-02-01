TEMPLATE = app
TARGET = ..\Demo\Demo
DEPENDPATH += . release
INCLUDEPATH += . ..\..\Libpedsim\src
DESTDIR = ..\

QMAKE_CXXFLAGS += -std=c++0x -g

LIBS +=  -L..\..\Debug -llibpedsim -lpthreadVC2 Qt5PlatformSupport.lib 

DEFINES += NOMINMAX

QT += opengl
QT += widgets

CONFIG += embed_manifest_exe
CONFIG += debug
CONFIG += console

# Input
HEADERS += MainWindow.h ParseScenario.h  ViewAgent.h PedSimulation.h
SOURCES += main.cpp MainWindow.cpp ParseScenario.cpp ViewAgent.cpp PedSimulation.cpp
