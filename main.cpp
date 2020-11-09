#include "mainwindow.h"
#include <QApplication>
#include "AVL.h"
#include <QPixmap>

using namespace std;
using namespace avl;
using namespace atl;

int main(int argc, char *argv[])
{

    //"/home/Downloads/road lines.jpg"





    QApplication a(argc, argv);
    MainWindow w;

    w.show();
    return a.exec();
}


