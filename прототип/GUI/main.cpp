#include <iostream>
#ifdef _DEBUG
  #undef _DEBUG
  #include <python.h>
  #define _DEBUG
#else
  #include <python.h>
#endif
#undef B0
#include <QApplication>
#include "mainwindow.h"
#include "QVTKWidget.h"
#include <QDebug>



int main(int argc, char *argv[]){

    QApplication a(argc, argv);


    MainWindow w(QString::fromUtf8(argv[0]));
    w.show();

    return a.exec();
}
