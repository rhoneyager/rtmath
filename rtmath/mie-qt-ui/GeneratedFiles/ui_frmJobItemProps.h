/********************************************************************************
** Form generated from reading UI file 'frmJobItemProps.ui'
**
** Created: Wed May 18 16:28:52 2011
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
#include <QtGui/QDialogButtonBox>
#include <QtGui/QFormLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPlainTextEdit>
#include <QtGui/QTextEdit>
#include <QtGui/QToolButton>

QT_BEGIN_NAMESPACE

class Ui_frmJobItemProp
{
public:
    QFormLayout *formLayout;
    QLabel *label;
    QLineEdit *txtName;
    QLabel *label_2;
    QTextEdit *txtDescription;
    QLabel *actionLabel;
    QComboBox *actionComboBox;
    QLabel *actionParametersLabel;
    QPlainTextEdit *txtActionParams;
    QToolButton *cmdModifyParams;
    QDialogButtonBox *buttonBox;

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

        actionLabel = new QLabel(frmJobItemProp);
        actionLabel->setObjectName(QString::fromUtf8("actionLabel"));

        formLayout->setWidget(2, QFormLayout::LabelRole, actionLabel);

        actionComboBox = new QComboBox(frmJobItemProp);
        actionComboBox->setObjectName(QString::fromUtf8("actionComboBox"));

        formLayout->setWidget(2, QFormLayout::FieldRole, actionComboBox);

        actionParametersLabel = new QLabel(frmJobItemProp);
        actionParametersLabel->setObjectName(QString::fromUtf8("actionParametersLabel"));

        formLayout->setWidget(3, QFormLayout::LabelRole, actionParametersLabel);

        txtActionParams = new QPlainTextEdit(frmJobItemProp);
        txtActionParams->setObjectName(QString::fromUtf8("txtActionParams"));
        txtActionParams->setReadOnly(true);

        formLayout->setWidget(3, QFormLayout::FieldRole, txtActionParams);

        cmdModifyParams = new QToolButton(frmJobItemProp);
        cmdModifyParams->setObjectName(QString::fromUtf8("cmdModifyParams"));

        formLayout->setWidget(4, QFormLayout::FieldRole, cmdModifyParams);

        buttonBox = new QDialogButtonBox(frmJobItemProp);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        formLayout->setWidget(5, QFormLayout::FieldRole, buttonBox);


        retranslateUi(frmJobItemProp);

        QMetaObject::connectSlotsByName(frmJobItemProp);
    } // setupUi

    void retranslateUi(QDialog *frmJobItemProp)
    {
        frmJobItemProp->setWindowTitle(QApplication::translate("frmJobItemProp", "Job Item Properties", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("frmJobItemProp", "Item Name:", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("frmJobItemProp", "Description:", 0, QApplication::UnicodeUTF8));
        actionLabel->setText(QApplication::translate("frmJobItemProp", "Action:", 0, QApplication::UnicodeUTF8));
        actionParametersLabel->setText(QApplication::translate("frmJobItemProp", "Action Parameters:", 0, QApplication::UnicodeUTF8));
        cmdModifyParams->setText(QApplication::translate("frmJobItemProp", "Modify Parameters...", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class frmJobItemProp: public Ui_frmJobItemProp {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FRMJOBITEMPROPS_H
