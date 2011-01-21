#include <boost/python.hpp>
#include <QtGui/QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    sim::MainWindow w;
    w.show();

    Py_Initialize();

    return a.exec();
}
