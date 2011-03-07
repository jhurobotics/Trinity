QT       += core gui opengl
CONFIG += debug
TARGET = sim
TEMPLATE = app

INCLUDEPATH += /usr/include/python2.6

SOURCES += \
    qt/mainwindow.cpp \
    qt/main.cpp \
    src/simulation.cpp \
    src/simsensors.cpp \
    src/simmotors.cpp \
    src/robot.cpp \
    src/map.cpp \
    src/geometry.cpp \
    src/timers.cpp \
    src/Nodel.cpp \
    src/serial.cpp \
    src/arduino.cpp \
    src/slam/OdometryModel.cpp \
    src/slam/mcl.cpp \
    src/slam/MeasurementModel.cpp \
    qt/mapwidget.cpp \
    qt/simwidget.cpp \
    src/bindings/py_robot.cpp \
    src/bindings/py_math.cpp

HEADERS  += \
    qt/mainwindow.h \
    src/simulation.h \
    src/simsensors.h \
    src/simmotors.h \
    src/robot.h \
    src/math.h \
    src/map.h \
    src/geometryio.h \
    src/geometry.h \
    src/graph.h \
    src/timers.h \
    src/serial.h \
    src/arduino.h \
    src/slam/slam.h \
    src/slam/simslam.h \
    qt/mapwidget.h \
    qt/simwidget.h

FORMS    += \
    qt/mainwindow.ui

LIBS  += -lpython2.6 -lboost_python
