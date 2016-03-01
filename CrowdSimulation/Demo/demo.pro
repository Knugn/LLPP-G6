Debug:message("Debug mode begin")
Release:message("Release mode begin")

TEMPLATE = app
TARGET = Demo
DEPENDPATH += ./src
Debug:TARGETDEPS += ../Libpedsim/Debug/Libpedsim.lib
Release:TARGETDEPS += ../Libpedsim/Release/Libpedsim.lib
VPATH += ./src
INCLUDEPATH += ./src ../Libpedsim/src
INCLUDEPATH += $$quote($$(CUDA_PATH)\include)

DESTDIR = ./Mixed
Release:DESTDIR = ./Release
Debug:DESTDIR = ./Debug


OBJECTS_DIR = $$DESTDIR/obj
MOC_DIR = $$DESTDIR/moc
RCC_DIR = $$DESTDIR/rcc
UI_DIR = $$DESTDIR/ui

QMAKE_CXXFLAGS += -std=c++0x -g

Debug:LIBS += -L../Libpedsim/Debug
Release:LIBS += -L../Libpedsim/Release
#LIBS += -L"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v7.5\lib\win32"
#LIBS += -L'"'$$(CudaToolkitLibDir)'"'
LIBS += -L'"'$$(CUDA_PATH)\lib\win32'"'
LIBS += -llibpedsim -lpthreadVC2 -lOpenCL
LIBS += Qt5PlatformSupport.lib 

DEFINES += NOMINMAX

QT += opengl
QT += widgets

CONFIG += embed_manifest_exe
#CONFIG += debug_and_release
#Debug:CONFIG += debug
#Release:CONFIG += release
CONFIG += console
CONFIG += c++11

# Input
HEADERS += MainWindow.h ParseScenario.h ViewAgent.h PedSimulation.h
SOURCES += main.cpp MainWindow.cpp ParseScenario.cpp ViewAgent.cpp PedSimulation.cpp

Debug:message("Debug mode end")
Release:message("Release mode end")
