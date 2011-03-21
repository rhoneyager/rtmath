#ifndef FRMLAYERPROPS_H
#define FRMLAYERPROPS_H

#include <QDialog>

namespace Ui {
    class frmLayerProps;
}

class frmLayerProps : public QDialog
{
    Q_OBJECT

public:
    explicit frmLayerProps(QWidget *parent = 0);
    ~frmLayerProps();

private:
    Ui::frmLayerProps *ui;
};

#endif // FRMLAYERPROPS_H
