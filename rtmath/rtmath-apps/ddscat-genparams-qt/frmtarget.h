#ifndef FRMTARGET_H
#define FRMTARGET_H

#include <QtGui/QMainWindow>
#include "ui_frmTarget.h"

class frmTarget : public QMainWindow
{
	Q_OBJECT

public:
	frmTarget(QWidget *parent = 0, Qt::WFlags flags = 0);
	~frmTarget();

private:
	Ui::frmTarget ui;
private slots:
	void processOK();
	void targetTypeChanged();
	void dimReInsChanged();
};

#endif // FRMTARGET_H
