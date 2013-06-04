#include "frmmain.h"
#include <QtWidgets/QApplication>

#pragma comment(lib, "tmatrix-cpp")
#pragma comment(lib, "tmatrix-fortran")

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	frmMain w;
	w.show();
	return a.exec();
}
