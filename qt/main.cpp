#include <QtGui/QApplication>
#include "mainwindow.h"
#include <stdlib.h>

#ifndef NODE_TEST

int main(int argc, char *argv[])
{
  srandom(time(NULL));
  QApplication a(argc, argv);
  sim::MainWindow w;
  w.show();
    
  return a.exec();
}

#endif // NODE_TEST
