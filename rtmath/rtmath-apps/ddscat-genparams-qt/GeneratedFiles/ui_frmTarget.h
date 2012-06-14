/********************************************************************************
** Form generated from reading UI file 'frmTarget.ui'
**
** Created: Tue Jun 12 23:16:15 2012
**      by: Qt User Interface Compiler version 4.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FRMTARGET_H
#define UI_FRMTARGET_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QFormLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QTextBrowser>
#include <QtGui/QToolButton>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_frmTarget
{
public:
    QVBoxLayout *verticalLayout;
    QFormLayout *formLayout;
    QLabel *label;
    QComboBox *cmbTargetType;
    QLabel *label_2;
    QHBoxLayout *horizontalLayout;
    QLineEdit *txtShapeBase;
    QToolButton *cmdBaseShapeSel;
    QLabel *label_3;
    QComboBox *cmbAspectRatio;
    QTextBrowser *textBrowser;
    QSpacerItem *verticalSpacer;
    QHBoxLayout *horizontalLayout_2;
    QSpacerItem *horizontalSpacer;
    QPushButton *cmdOK;
    QPushButton *cmdCancel;

    void setupUi(QWidget *frmTarget)
    {
        if (frmTarget->objectName().isEmpty())
            frmTarget->setObjectName(QString::fromUtf8("frmTarget"));
        frmTarget->resize(393, 362);
        verticalLayout = new QVBoxLayout(frmTarget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        formLayout = new QFormLayout();
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        label = new QLabel(frmTarget);
        label->setObjectName(QString::fromUtf8("label"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label);

        cmbTargetType = new QComboBox(frmTarget);
        cmbTargetType->setObjectName(QString::fromUtf8("cmbTargetType"));

        formLayout->setWidget(0, QFormLayout::FieldRole, cmbTargetType);

        label_2 = new QLabel(frmTarget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label_2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        txtShapeBase = new QLineEdit(frmTarget);
        txtShapeBase->setObjectName(QString::fromUtf8("txtShapeBase"));

        horizontalLayout->addWidget(txtShapeBase);

        cmdBaseShapeSel = new QToolButton(frmTarget);
        cmdBaseShapeSel->setObjectName(QString::fromUtf8("cmdBaseShapeSel"));

        horizontalLayout->addWidget(cmdBaseShapeSel);


        formLayout->setLayout(1, QFormLayout::FieldRole, horizontalLayout);

        label_3 = new QLabel(frmTarget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        formLayout->setWidget(2, QFormLayout::LabelRole, label_3);

        cmbAspectRatio = new QComboBox(frmTarget);
        cmbAspectRatio->setObjectName(QString::fromUtf8("cmbAspectRatio"));

        formLayout->setWidget(2, QFormLayout::FieldRole, cmbAspectRatio);

        textBrowser = new QTextBrowser(frmTarget);
        textBrowser->setObjectName(QString::fromUtf8("textBrowser"));

        formLayout->setWidget(3, QFormLayout::FieldRole, textBrowser);


        verticalLayout->addLayout(formLayout);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setSizeConstraint(QLayout::SetMinAndMaxSize);
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);

        cmdOK = new QPushButton(frmTarget);
        cmdOK->setObjectName(QString::fromUtf8("cmdOK"));
        cmdOK->setAutoDefault(true);
        cmdOK->setDefault(true);

        horizontalLayout_2->addWidget(cmdOK);

        cmdCancel = new QPushButton(frmTarget);
        cmdCancel->setObjectName(QString::fromUtf8("cmdCancel"));

        horizontalLayout_2->addWidget(cmdCancel);


        verticalLayout->addLayout(horizontalLayout_2);


        retranslateUi(frmTarget);

        QMetaObject::connectSlotsByName(frmTarget);
    } // setupUi

    void retranslateUi(QWidget *frmTarget)
    {
        frmTarget->setWindowTitle(QApplication::translate("frmTarget", "Target Properties", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("frmTarget", "Target Type:", 0, QApplication::UnicodeUTF8));
        cmbTargetType->clear();
        cmbTargetType->insertItems(0, QStringList()
         << QApplication::translate("frmTarget", "From Base Shape File", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmTarget", "Ellipsoid", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmTarget", "Hex Plate", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmTarget", "Rectangular Prism", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmTarget", "Column / Needle", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmTarget", "TMATRIX Ellipsoid", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmTarget", "TMATRIX Cylinder", 0, QApplication::UnicodeUTF8)
        );
        label_2->setText(QApplication::translate("frmTarget", "Base Shape File:", 0, QApplication::UnicodeUTF8));
        cmdBaseShapeSel->setText(QApplication::translate("frmTarget", "...", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("frmTarget", "Dimension Relations:", 0, QApplication::UnicodeUTF8));
        cmbAspectRatio->clear();
        cmbAspectRatio->insertItems(0, QStringList()
         << QApplication::translate("frmTarget", "No relation", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmTarget", "Wyser et al. (1998)", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmTarget", "Perturbed Wyser", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmTarget", "Custom", 0, QApplication::UnicodeUTF8)
        );
        textBrowser->setHtml(QApplication::translate("frmTarget", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:8pt;\">Relation info goes here...</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        cmdOK->setText(QApplication::translate("frmTarget", "OK", 0, QApplication::UnicodeUTF8));
        cmdCancel->setText(QApplication::translate("frmTarget", "Cancel", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class frmTarget: public Ui_frmTarget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FRMTARGET_H
