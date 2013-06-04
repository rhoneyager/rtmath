#include "stdafx.h"
#include "frmlayerprops.h"
#include "ui_frmlayerprops.h"

frmLayerProps::frmLayerProps(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::frmLayerProps)
{
    ui->setupUi(this);
}

frmLayerProps::~frmLayerProps()
{
    delete ui;
}
