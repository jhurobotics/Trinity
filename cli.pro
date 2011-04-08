QT       -= core gui
CONFIG += debug
TARGET = cli
TEMPLATE = app
DEFINES += NO_GUI

SOURCES += \
    src/simulation.cpp \
    src/simsensors.cpp \
    src/simmotors.cpp \
    src/robot.cpp \
    src/map.cpp \
    src/geometry.cpp \
    src/timers.cpp \
    src/graph.cpp \
    src/serial.cpp \
    src/arduino.cpp \
    src/maestro.cpp \
    src/setup.cpp \
    src/cli.cpp \
    src/speedTest.cpp \
    src/slam/OdometryModel.cpp \
    src/slam/mcl.cpp \
    src/slam/MeasurementModel.cpp \

HEADERS  += \
    src/simulation.h \
    src/simsensors.h \
    src/simmotors.h \
    src/robot.h \
    src/math.h \
    src/map.h \
    src/geometryio.h \
    src/geometry.h \
    src/timers.h \
    src/graph.h \
    src/serial.h \
    src/arduino.h \
    src/controllers.h \
    src/speedTest.h \
    src/slam/slam.h \
    src/slam/simslam.h \
