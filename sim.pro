#-------------------------------------------------
#
# Project created by QtCreator 2011-01-03T23:06:37
#
#-------------------------------------------------

QT       += core gui opengl

TARGET = sim
TEMPLATE = app


SOURCES += \
    qt/mainwindow.cpp \
    qt/main.cpp \
    src/simulation.cpp \
    src/simsensors.cpp \
    src/simmotors.cpp \
    src/robot.cpp \
    src/map.cpp \
    src/geometry.cpp

HEADERS  += \
    qt/mainwindow.h \
    src/simulation.h \
    src/simsensors.h \
    src/simmotors.h \
    src/robot.h \
    src/math.h \
    src/map.h \
    src/geometryio.h \
    src/geometry.h

FORMS    += \
    qt/mainwindow.ui
