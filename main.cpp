#include "mainwindow.h"
#include <QApplication>
#include "AVL.h"
#include <QPixmap>


int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    MainWindow w;

    w.show();
    return a.exec();
}


