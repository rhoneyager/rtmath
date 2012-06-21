#include "stdafx.h"
#include "frmmain.h"
//#include <QtCore>
#include <QApplication>
//#include <QtGui/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	frmMain w;
	w.show();
	return a.exec();
}
