/********************************************************************************
** Form generated from reading UI file 'frmlayerprops.ui'
**
** Created: Mon Mar 21 14:22:39 2011
**      by: Qt User Interface Compiler version 4.7.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FRMLAYERPROPS_H
#define UI_FRMLAYERPROPS_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QHeaderView>

QT_BEGIN_NAMESPACE

class Ui_frmLayerProps
{
public:
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *frmLayerProps)
    {
        if (frmLayerProps->objectName().isEmpty())
            frmLayerProps->setObjectName(QString::fromUtf8("frmLayerProps"));
        frmLayerProps->resize(500, 400);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/rtlogo"), QSize(), QIcon::Normal, QIcon::Off);
        frmLayerProps->setWindowIcon(icon);
        buttonBox = new QDialogButtonBox(frmLayerProps);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setGeometry(QRect(10, 360, 481, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        retranslateUi(frmLayerProps);
        QObject::connect(buttonBox, SIGNAL(accepted()), frmLayerProps, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), frmLayerProps, SLOT(reject()));

        QMetaObject::connectSlotsByName(frmLayerProps);
    } // setupUi

    void retranslateUi(QDialog *frmLayerProps)
    {
        frmLayerProps->setWindowTitle(QApplication::translate("frmLayerProps", "Dialog", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class frmLayerProps: public Ui_frmLayerProps {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FRMLAYERPROPS_H
