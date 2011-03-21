#ifndef FRMMODELPROPS_H
#define FRMMODELPROPS_H

#include <QDialog>

namespace Ui {
    class frmModelProps;
}

class frmModelProps : public QDialog
{
    Q_OBJECT

public:
    explicit frmModelProps(QWidget *parent = 0);
    ~frmModelProps();

private:
    Ui::frmModelProps *ui;
};

#endif // FRMMODELPROPS_H
