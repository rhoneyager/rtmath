/********************************************************************************
** Form generated from reading UI file 'frmmodelprops.ui'
**
** Created: Mon Mar 21 14:22:39 2011
**      by: Qt User Interface Compiler version 4.7.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FRMMODELPROPS_H
#define UI_FRMMODELPROPS_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QDateTimeEdit>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QFormLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QTextEdit>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_frmModelProps
{
public:
    QWidget *formLayoutWidget;
    QFormLayout *formLayout;
    QLabel *modelNameLabel;
    QLineEdit *modelNameLineEdit;
    QLabel *modelDescriptionLabel;
    QTextEdit *textEdit;
    QLabel *baseAtmosphereLabel;
    QComboBox *baseAtmosphereComboBox;
    QLabel *atmosphericOverlaysLabel;
    QComboBox *atmosphericOverlaysComboBox;
    QLabel *surfaceLabel;
    QComboBox *surfaceComboBox;
    QLabel *illuminationLabel;
    QComboBox *illuminationComboBox;
    QLabel *filtersLabel;
    QComboBox *filtersComboBox;
    QLabel *createdLabel;
    QDateTimeEdit *createdDateTimeEdit;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *frmModelProps)
    {
        if (frmModelProps->objectName().isEmpty())
            frmModelProps->setObjectName(QString::fromUtf8("frmModelProps"));
        frmModelProps->resize(400, 350);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(frmModelProps->sizePolicy().hasHeightForWidth());
        frmModelProps->setSizePolicy(sizePolicy);
        frmModelProps->setMinimumSize(QSize(400, 350));
        frmModelProps->setMaximumSize(QSize(400, 350));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/rtlogo"), QSize(), QIcon::Normal, QIcon::Off);
        frmModelProps->setWindowIcon(icon);
        formLayoutWidget = new QWidget(frmModelProps);
        formLayoutWidget->setObjectName(QString::fromUtf8("formLayoutWidget"));
        formLayoutWidget->setGeometry(QRect(10, 10, 381, 301));
        formLayout = new QFormLayout(formLayoutWidget);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        formLayout->setSizeConstraint(QLayout::SetMinAndMaxSize);
        formLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        formLayout->setContentsMargins(0, 0, 0, 0);
        modelNameLabel = new QLabel(formLayoutWidget);
        modelNameLabel->setObjectName(QString::fromUtf8("modelNameLabel"));

        formLayout->setWidget(0, QFormLayout::LabelRole, modelNameLabel);

        modelNameLineEdit = new QLineEdit(formLayoutWidget);
        modelNameLineEdit->setObjectName(QString::fromUtf8("modelNameLineEdit"));

        formLayout->setWidget(0, QFormLayout::FieldRole, modelNameLineEdit);

        modelDescriptionLabel = new QLabel(formLayoutWidget);
        modelDescriptionLabel->setObjectName(QString::fromUtf8("modelDescriptionLabel"));

        formLayout->setWidget(1, QFormLayout::LabelRole, modelDescriptionLabel);

        textEdit = new QTextEdit(formLayoutWidget);
        textEdit->setObjectName(QString::fromUtf8("textEdit"));
        textEdit->setTabChangesFocus(true);

        formLayout->setWidget(1, QFormLayout::FieldRole, textEdit);

        baseAtmosphereLabel = new QLabel(formLayoutWidget);
        baseAtmosphereLabel->setObjectName(QString::fromUtf8("baseAtmosphereLabel"));

        formLayout->setWidget(2, QFormLayout::LabelRole, baseAtmosphereLabel);

        baseAtmosphereComboBox = new QComboBox(formLayoutWidget);
        baseAtmosphereComboBox->setObjectName(QString::fromUtf8("baseAtmosphereComboBox"));

        formLayout->setWidget(2, QFormLayout::FieldRole, baseAtmosphereComboBox);

        atmosphericOverlaysLabel = new QLabel(formLayoutWidget);
        atmosphericOverlaysLabel->setObjectName(QString::fromUtf8("atmosphericOverlaysLabel"));

        formLayout->setWidget(3, QFormLayout::LabelRole, atmosphericOverlaysLabel);

        atmosphericOverlaysComboBox = new QComboBox(formLayoutWidget);
        atmosphericOverlaysComboBox->setObjectName(QString::fromUtf8("atmosphericOverlaysComboBox"));

        formLayout->setWidget(3, QFormLayout::FieldRole, atmosphericOverlaysComboBox);

        surfaceLabel = new QLabel(formLayoutWidget);
        surfaceLabel->setObjectName(QString::fromUtf8("surfaceLabel"));

        formLayout->setWidget(4, QFormLayout::LabelRole, surfaceLabel);

        surfaceComboBox = new QComboBox(formLayoutWidget);
        surfaceComboBox->setObjectName(QString::fromUtf8("surfaceComboBox"));

        formLayout->setWidget(4, QFormLayout::FieldRole, surfaceComboBox);

        illuminationLabel = new QLabel(formLayoutWidget);
        illuminationLabel->setObjectName(QString::fromUtf8("illuminationLabel"));

        formLayout->setWidget(5, QFormLayout::LabelRole, illuminationLabel);

        illuminationComboBox = new QComboBox(formLayoutWidget);
        illuminationComboBox->setObjectName(QString::fromUtf8("illuminationComboBox"));

        formLayout->setWidget(5, QFormLayout::FieldRole, illuminationComboBox);

        filtersLabel = new QLabel(formLayoutWidget);
        filtersLabel->setObjectName(QString::fromUtf8("filtersLabel"));

        formLayout->setWidget(6, QFormLayout::LabelRole, filtersLabel);

        filtersComboBox = new QComboBox(formLayoutWidget);
        filtersComboBox->setObjectName(QString::fromUtf8("filtersComboBox"));

        formLayout->setWidget(6, QFormLayout::FieldRole, filtersComboBox);

        createdLabel = new QLabel(formLayoutWidget);
        createdLabel->setObjectName(QString::fromUtf8("createdLabel"));

        formLayout->setWidget(7, QFormLayout::LabelRole, createdLabel);

        createdDateTimeEdit = new QDateTimeEdit(formLayoutWidget);
        createdDateTimeEdit->setObjectName(QString::fromUtf8("createdDateTimeEdit"));
        createdDateTimeEdit->setReadOnly(true);
        createdDateTimeEdit->setButtonSymbols(QAbstractSpinBox::UpDownArrows);
        createdDateTimeEdit->setCalendarPopup(true);

        formLayout->setWidget(7, QFormLayout::FieldRole, createdDateTimeEdit);

        buttonBox = new QDialogButtonBox(frmModelProps);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setGeometry(QRect(230, 320, 156, 23));
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        retranslateUi(frmModelProps);
        QObject::connect(buttonBox, SIGNAL(rejected()), frmModelProps, SLOT(close()));

        QMetaObject::connectSlotsByName(frmModelProps);
    } // setupUi

    void retranslateUi(QDialog *frmModelProps)
    {
        frmModelProps->setWindowTitle(QApplication::translate("frmModelProps", "Model Properties", 0, QApplication::UnicodeUTF8));
        modelNameLabel->setText(QApplication::translate("frmModelProps", "Model Name:", 0, QApplication::UnicodeUTF8));
        modelDescriptionLabel->setText(QApplication::translate("frmModelProps", "Model Description:", 0, QApplication::UnicodeUTF8));
        baseAtmosphereLabel->setText(QApplication::translate("frmModelProps", "Base Atmosphere:", 0, QApplication::UnicodeUTF8));
        atmosphericOverlaysLabel->setText(QApplication::translate("frmModelProps", "Atmospheric Overlays:", 0, QApplication::UnicodeUTF8));
        surfaceLabel->setText(QApplication::translate("frmModelProps", "Surface:", 0, QApplication::UnicodeUTF8));
        illuminationLabel->setText(QApplication::translate("frmModelProps", "Illumination", 0, QApplication::UnicodeUTF8));
        filtersLabel->setText(QApplication::translate("frmModelProps", "Filters:", 0, QApplication::UnicodeUTF8));
        createdLabel->setText(QApplication::translate("frmModelProps", "Created:", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class frmModelProps: public Ui_frmModelProps {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FRMMODELPROPS_H
