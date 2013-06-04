#include "stdafx.h"
#include <QtGui/QApplication>
#include "frmmain.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    frmMain w;
    w.show();

    return a.exec();
}
