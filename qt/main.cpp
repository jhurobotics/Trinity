#include <boost/python.hpp>
#include <QtGui/QApplication>
#include "mainwindow.h"

#ifndef NODE_TEST

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    sim::MainWindow w;
    w.show();

    Py_Initialize();

    return a.exec();
}

#endif NODE_TEST