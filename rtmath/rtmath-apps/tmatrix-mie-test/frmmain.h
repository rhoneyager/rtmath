#ifndef FRMMAIN_H
#define FRMMAIN_H

#include <QtWidgets/QMainWindow>
#include <boost/shared_ptr.hpp>
#include <tmatrix/tmatrix.h>
#include "ui_frmmain.h"

class frmMain : public QMainWindow
{
	Q_OBJECT

public:
	frmMain(QWidget *parent = 0);
	~frmMain();
	public slots:
		void commit();
		void import();
private:
	Ui::frmMainClass ui;
	boost::shared_ptr<const tmatrix::tmatrixParams> tp;
	boost::shared_ptr<const tmatrix::OriTmatrix> to;
	boost::shared_ptr<const tmatrix::OriAngleRes> ta;
};

#endif // FRMMAIN_H
