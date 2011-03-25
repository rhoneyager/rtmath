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

	public slots:
		void jobRun();
		void jobItemAdd();
		void jobItemDelete();
		void jobItemModify();
		void jobNew();
		void jobOpen();
		void jobSave();
		void jobSaveOutput();
		void jobExportImage();
		void imageProperties();
};

#endif // FRMMAIN_H
