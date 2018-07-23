#include "frmmain.h"
#include <QtGui/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	frmmain w;
	w.show();
	return a.exec();
}
