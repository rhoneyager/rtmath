#include "stdafx.h"
#include "frmmodelprops.h"
#include "ui_frmmodelprops.h"

frmModelProps::frmModelProps(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::frmModelProps)
{
    ui->setupUi(this);
}

frmModelProps::~frmModelProps()
{
    delete ui;
}
