QT       += core gui opengl

TARGET = sim
TEMPLATE = app

INCLUDEPATH += /usr/local/include/python2.7

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
    src/slam/OdometryModel.cpp \
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
    src/slam.h \
    src/timers.h \
    qt/mapwidget.h \
    qt/simwidget.h

FORMS    += \
    qt/mainwindow.ui

LIBS  += -lpython2.7 -lboost_python-mt
