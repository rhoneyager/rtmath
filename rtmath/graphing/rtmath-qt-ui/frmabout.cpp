#include "stdafx.h"
#include "frmabout.h"
#include "ui_frmabout.h"

frmAbout::frmAbout(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::frmAbout)
{
    ui->setupUi(this);
    //ui->graphicsView->
}

frmAbout::~frmAbout()
{
    delete ui;
}
