#include "stdafx.h"
#include "frmmain.h"
#include "ui_frmmain.h"

#include "frmabout.h"
#include "frmmodelprops.h"
#include <QFileDialog>

frmMain::frmMain(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::frmMain)
{
    ui->setupUi(this);
	ui->lblRev->setText(tr("This is a test"));
	//ui->lblRev
}

frmMain::~frmMain()
{
    delete ui;
}

void frmMain::model_props()
{
    frmModelProps props;
    props.exec();
}

void frmMain::show_about()
{
    frmAbout newfrm;
    //newfrm.show();
    newfrm.exec();
}

void frmMain::atmos_open()
{
    QString fileName;
    fileName = QFileDialog::getOpenFileName(this,tr("Select Atmospheric Parameters File"), tr(""), tr("Atmospheres (*.atmos)"));
}
