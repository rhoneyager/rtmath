/********************************************************************************
** Form generated from reading UI file 'frmJobItemProps.ui'
**
** Created: Thu Mar 24 17:01:55 2011
**      by: Qt User Interface Compiler version 4.7.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FRMJOBITEMPROPS_H
#define UI_FRMJOBITEMPROPS_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QFormLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QTextEdit>

QT_BEGIN_NAMESPACE

class Ui_frmJobItemProp
{
public:
    QFormLayout *formLayout;
    QLabel *label;
    QLineEdit *txtName;
    QLabel *label_2;
    QTextEdit *txtDescription;
    QComboBox *actionComboBox;
    QLabel *actionLabel;
    QGroupBox *groupBox;

    void setupUi(QDialog *frmJobItemProp)
    {
        if (frmJobItemProp->objectName().isEmpty())
            frmJobItemProp->setObjectName(QString::fromUtf8("frmJobItemProp"));
        frmJobItemProp->resize(584, 451);
        formLayout = new QFormLayout(frmJobItemProp);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        formLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        label = new QLabel(frmJobItemProp);
        label->setObjectName(QString::fromUtf8("label"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label);

        txtName = new QLineEdit(frmJobItemProp);
        txtName->setObjectName(QString::fromUtf8("txtName"));

        formLayout->setWidget(0, QFormLayout::FieldRole, txtName);

        label_2 = new QLabel(frmJobItemProp);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label_2);

        txtDescription = new QTextEdit(frmJobItemProp);
        txtDescription->setObjectName(QString::fromUtf8("txtDescription"));

        formLayout->setWidget(1, QFormLayout::FieldRole, txtDescription);

        actionComboBox = new QComboBox(frmJobItemProp);
        actionComboBox->setObjectName(QString::fromUtf8("actionComboBox"));

        formLayout->setWidget(2, QFormLayout::FieldRole, actionComboBox);

        actionLabel = new QLabel(frmJobItemProp);
        actionLabel->setObjectName(QString::fromUtf8("actionLabel"));

        formLayout->setWidget(2, QFormLayout::LabelRole, actionLabel);

        groupBox = new QGroupBox(frmJobItemProp);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));

        formLayout->setWidget(3, QFormLayout::FieldRole, groupBox);


        retranslateUi(frmJobItemProp);

        QMetaObject::connectSlotsByName(frmJobItemProp);
    } // setupUi

    void retranslateUi(QDialog *frmJobItemProp)
    {
        frmJobItemProp->setWindowTitle(QApplication::translate("frmJobItemProp", "Job Item Properties", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("frmJobItemProp", "Item Name:", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("frmJobItemProp", "Description:", 0, QApplication::UnicodeUTF8));
        actionLabel->setText(QApplication::translate("frmJobItemProp", "Action:", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("frmJobItemProp", "Action Parameters:", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class frmJobItemProp: public Ui_frmJobItemProp {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FRMJOBITEMPROPS_H
