/********************************************************************************
** Form generated from reading UI file 'frmmain.ui'
**
** Created: Thu Aug 16 01:46:42 2012
**      by: Qt User Interface Compiler version 4.7.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FRMMAIN_H
#define UI_FRMMAIN_H

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
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_frmMainClass
{
public:
    QAction *action_Quit;
    QWidget *centralWidget;
    QFormLayout *formLayout;
    QLabel *label;
    QHBoxLayout *horizontalLayout;
    QLineEdit *txtBaseDir;
    QPushButton *cmdBaseFind;
    QLabel *label_6;
    QLabel *label_7;
    QComboBox *cmbShapePattern;
    QLabel *label_2;
    QLineEdit *txtTemp;
    QLabel *label_5;
    QComboBox *cmbShapeMeth;
    QLabel *label_3;
    QLabel *label_4;
    QComboBox *cmbVolFrac;
    QPushButton *cmdGenerate;
    QHBoxLayout *horizontalLayout_2;
    QLineEdit *txtDefaultPar;
    QPushButton *cmdDefFind;
    QHBoxLayout *horizontalLayout_3;
    QComboBox *cmbDiel;
    QLineEdit *txtDiel;
    QMenuBar *menuBar;
    QMenu *menu_File;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *frmMainClass)
    {
        if (frmMainClass->objectName().isEmpty())
            frmMainClass->setObjectName(QString::fromUtf8("frmMainClass"));
        frmMainClass->resize(624, 280);
        frmMainClass->setMinimumSize(QSize(0, 280));
        frmMainClass->setMaximumSize(QSize(16777215, 280));
        action_Quit = new QAction(frmMainClass);
        action_Quit->setObjectName(QString::fromUtf8("action_Quit"));
        centralWidget = new QWidget(frmMainClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        formLayout = new QFormLayout(centralWidget);
        formLayout->setSpacing(6);
        formLayout->setContentsMargins(11, 11, 11, 11);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        formLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        txtBaseDir = new QLineEdit(centralWidget);
        txtBaseDir->setObjectName(QString::fromUtf8("txtBaseDir"));

        horizontalLayout->addWidget(txtBaseDir);

        cmdBaseFind = new QPushButton(centralWidget);
        cmdBaseFind->setObjectName(QString::fromUtf8("cmdBaseFind"));

        horizontalLayout->addWidget(cmdBaseFind);


        formLayout->setLayout(0, QFormLayout::FieldRole, horizontalLayout);

        label_6 = new QLabel(centralWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label_6);

        label_7 = new QLabel(centralWidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        formLayout->setWidget(2, QFormLayout::LabelRole, label_7);

        cmbShapePattern = new QComboBox(centralWidget);
        cmbShapePattern->setObjectName(QString::fromUtf8("cmbShapePattern"));

        formLayout->setWidget(2, QFormLayout::FieldRole, cmbShapePattern);

        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        formLayout->setWidget(3, QFormLayout::LabelRole, label_2);

        txtTemp = new QLineEdit(centralWidget);
        txtTemp->setObjectName(QString::fromUtf8("txtTemp"));

        formLayout->setWidget(3, QFormLayout::FieldRole, txtTemp);

        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        formLayout->setWidget(4, QFormLayout::LabelRole, label_5);

        cmbShapeMeth = new QComboBox(centralWidget);
        cmbShapeMeth->setObjectName(QString::fromUtf8("cmbShapeMeth"));

        formLayout->setWidget(4, QFormLayout::FieldRole, cmbShapeMeth);

        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        formLayout->setWidget(5, QFormLayout::LabelRole, label_3);

        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        formLayout->setWidget(6, QFormLayout::LabelRole, label_4);

        cmbVolFrac = new QComboBox(centralWidget);
        cmbVolFrac->setObjectName(QString::fromUtf8("cmbVolFrac"));

        formLayout->setWidget(6, QFormLayout::FieldRole, cmbVolFrac);

        cmdGenerate = new QPushButton(centralWidget);
        cmdGenerate->setObjectName(QString::fromUtf8("cmdGenerate"));

        formLayout->setWidget(7, QFormLayout::FieldRole, cmdGenerate);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        txtDefaultPar = new QLineEdit(centralWidget);
        txtDefaultPar->setObjectName(QString::fromUtf8("txtDefaultPar"));

        horizontalLayout_2->addWidget(txtDefaultPar);

        cmdDefFind = new QPushButton(centralWidget);
        cmdDefFind->setObjectName(QString::fromUtf8("cmdDefFind"));

        horizontalLayout_2->addWidget(cmdDefFind);


        formLayout->setLayout(1, QFormLayout::FieldRole, horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        cmbDiel = new QComboBox(centralWidget);
        cmbDiel->setObjectName(QString::fromUtf8("cmbDiel"));

        horizontalLayout_3->addWidget(cmbDiel);

        txtDiel = new QLineEdit(centralWidget);
        txtDiel->setObjectName(QString::fromUtf8("txtDiel"));

        horizontalLayout_3->addWidget(txtDiel);


        formLayout->setLayout(5, QFormLayout::FieldRole, horizontalLayout_3);

        frmMainClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(frmMainClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 624, 21));
        menu_File = new QMenu(menuBar);
        menu_File->setObjectName(QString::fromUtf8("menu_File"));
        frmMainClass->setMenuBar(menuBar);
        statusBar = new QStatusBar(frmMainClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        frmMainClass->setStatusBar(statusBar);

        menuBar->addAction(menu_File->menuAction());
        menu_File->addAction(action_Quit);

        retranslateUi(frmMainClass);
        QObject::connect(action_Quit, SIGNAL(activated()), frmMainClass, SLOT(close()));
        QObject::connect(cmdGenerate, SIGNAL(clicked()), frmMainClass, SLOT(doGenerate()));
        QObject::connect(cmdDefFind, SIGNAL(clicked()), frmMainClass, SLOT(findDefaultPar()));
        QObject::connect(cmdBaseFind, SIGNAL(clicked()), frmMainClass, SLOT(findBaseDir()));

        QMetaObject::connectSlotsByName(frmMainClass);
    } // setupUi

    void retranslateUi(QMainWindow *frmMainClass)
    {
        frmMainClass->setWindowTitle(QApplication::translate("frmMainClass", "T-MATRIX In-Place Converter", 0, QApplication::UnicodeUTF8));
        action_Quit->setText(QApplication::translate("frmMainClass", "&Quit", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("frmMainClass", "Base Directory:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        txtBaseDir->setStatusTip(QApplication::translate("frmMainClass", "Specify base directory containing all of the DDSCAT runs. Recursion is assumed.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        cmdBaseFind->setText(QApplication::translate("frmMainClass", "...", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("frmMainClass", "Default ddscat.par file:", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("frmMainClass", "Shape File Pattern:", 0, QApplication::UnicodeUTF8));
        cmbShapePattern->clear();
        cmbShapePattern->insertItems(0, QStringList()
         << QApplication::translate("frmMainClass", "shape.dat", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmMainClass", "*.shp", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmMainClass", "*.txt", 0, QApplication::UnicodeUTF8)
        );
#ifndef QT_NO_STATUSTIP
        cmbShapePattern->setStatusTip(QApplication::translate("frmMainClass", "Specify the pattern of the autodetected shape.dat files.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        label_2->setText(QApplication::translate("frmMainClass", "Temperature (K):", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        txtTemp->setStatusTip(QApplication::translate("frmMainClass", "Specify the temperature (in Kelvin)", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        txtTemp->setPlaceholderText(QApplication::translate("frmMainClass", "263", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("frmMainClass", "Shape Dimensioning:", 0, QApplication::UnicodeUTF8));
        cmbShapeMeth->clear();
        cmbShapeMeth->insertItems(0, QStringList()
         << QApplication::translate("frmMainClass", "Same RMS aspect ratio", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmMainClass", "Same real aspect ratio", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmMainClass", "Equiv Aeff Sphere", 0, QApplication::UnicodeUTF8)
        );
#ifndef QT_NO_STATUSTIP
        cmbShapeMeth->setStatusTip(QApplication::translate("frmMainClass", "Select the method for approximating the shape file", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        label_3->setText(QApplication::translate("frmMainClass", "Dielectric Method:", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("frmMainClass", "Volume Fraction Method:", 0, QApplication::UnicodeUTF8));
        cmbVolFrac->clear();
        cmbVolFrac->insertItems(0, QStringList()
         << QApplication::translate("frmMainClass", "Minimal circumscribing sphere", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmMainClass", "Convex hull", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmMainClass", "Max Ellipsoid", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmMainClass", "RMS Ellipsoid", 0, QApplication::UnicodeUTF8)
        );
#ifndef QT_NO_STATUSTIP
        cmbVolFrac->setStatusTip(QApplication::translate("frmMainClass", "Specify the volume fraction method used in the dielectric calculations.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
#ifndef QT_NO_STATUSTIP
        cmdGenerate->setStatusTip(QApplication::translate("frmMainClass", "Generate the run definitions and statistics.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        cmdGenerate->setText(QApplication::translate("frmMainClass", "Generate Batch Run Files and Base Statistics", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        txtDefaultPar->setStatusTip(QApplication::translate("frmMainClass", "The default file is used if a ddscat.par file cannot be detected in the same dir as the shape.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        cmdDefFind->setText(QApplication::translate("frmMainClass", "...", 0, QApplication::UnicodeUTF8));
        cmbDiel->clear();
        cmbDiel->insertItems(0, QStringList()
         << QApplication::translate("frmMainClass", "Sihvola", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmMainClass", "Debye", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmMainClass", "Maxwell-Garnett", 0, QApplication::UnicodeUTF8)
        );
#ifndef QT_NO_STATUSTIP
        cmbDiel->setStatusTip(QApplication::translate("frmMainClass", "Specify the dielectric constant calculation method.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
#ifndef QT_NO_STATUSTIP
        txtDiel->setStatusTip(QApplication::translate("frmMainClass", "Value of nu for the Sihvola method", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        txtDiel->setPlaceholderText(QApplication::translate("frmMainClass", "0.85", 0, QApplication::UnicodeUTF8));
        menu_File->setTitle(QApplication::translate("frmMainClass", "&File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class frmMainClass: public Ui_frmMainClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FRMMAIN_H
