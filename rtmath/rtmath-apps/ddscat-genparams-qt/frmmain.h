#ifndef FRMMAIN_H
#define FRMMAIN_H

#include <QtGui/QMainWindow>
#include "ui_frmmain.h"

class frmMain : public QMainWindow
{
	Q_OBJECT

public:
	frmMain(QWidget *parent = 0, Qt::WFlags flags = 0);
	~frmMain();

private:
	Ui::frmMainClass ui;
};

#endif // FRMMAIN_H
