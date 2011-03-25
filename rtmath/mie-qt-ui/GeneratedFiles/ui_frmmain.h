/********************************************************************************
** Form generated from reading UI file 'frmmain.ui'
**
** Created: Thu Mar 24 17:01:55 2011
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
#include <QtGui/QColumnView>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_frmMainClass
{
public:
    QAction *action_New;
    QAction *action_Open;
    QAction *action_Save;
    QAction *actionE_xit;
    QAction *actionWn_Calculation;
    QAction *actionTau_Calculation;
    QAction *actionPi_Calculation;
    QAction *actionA_b_Calculation;
    QAction *actionExtinction;
    QAction *actionAbsorption;
    QAction *actionScattering;
    QAction *actionSave_Output;
    QAction *action_Properties;
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QTabWidget *tabWidget;
    QWidget *tab;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QColumnView *columnView;
    QHBoxLayout *horizontalLayout;
    QPushButton *cmdAddjob;
    QPushButton *cmddeljob;
    QPushButton *cmdProperties;
    QPushButton *cmdjobRun;
    QWidget *tab_3;
    QWidget *tab_2;
    QMenuBar *menuBar;
    QMenu *menu_File;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *frmMainClass)
    {
        if (frmMainClass->objectName().isEmpty())
            frmMainClass->setObjectName(QString::fromUtf8("frmMainClass"));
        frmMainClass->resize(714, 646);
        action_New = new QAction(frmMainClass);
        action_New->setObjectName(QString::fromUtf8("action_New"));
        action_Open = new QAction(frmMainClass);
        action_Open->setObjectName(QString::fromUtf8("action_Open"));
        action_Save = new QAction(frmMainClass);
        action_Save->setObjectName(QString::fromUtf8("action_Save"));
        actionE_xit = new QAction(frmMainClass);
        actionE_xit->setObjectName(QString::fromUtf8("actionE_xit"));
        actionWn_Calculation = new QAction(frmMainClass);
        actionWn_Calculation->setObjectName(QString::fromUtf8("actionWn_Calculation"));
        actionTau_Calculation = new QAction(frmMainClass);
        actionTau_Calculation->setObjectName(QString::fromUtf8("actionTau_Calculation"));
        actionPi_Calculation = new QAction(frmMainClass);
        actionPi_Calculation->setObjectName(QString::fromUtf8("actionPi_Calculation"));
        actionA_b_Calculation = new QAction(frmMainClass);
        actionA_b_Calculation->setObjectName(QString::fromUtf8("actionA_b_Calculation"));
        actionExtinction = new QAction(frmMainClass);
        actionExtinction->setObjectName(QString::fromUtf8("actionExtinction"));
        actionAbsorption = new QAction(frmMainClass);
        actionAbsorption->setObjectName(QString::fromUtf8("actionAbsorption"));
        actionScattering = new QAction(frmMainClass);
        actionScattering->setObjectName(QString::fromUtf8("actionScattering"));
        actionSave_Output = new QAction(frmMainClass);
        actionSave_Output->setObjectName(QString::fromUtf8("actionSave_Output"));
        action_Properties = new QAction(frmMainClass);
        action_Properties->setObjectName(QString::fromUtf8("action_Properties"));
        centralWidget = new QWidget(frmMainClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        verticalLayout_2 = new QVBoxLayout(tab);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label = new QLabel(tab);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout->addWidget(label);

        columnView = new QColumnView(tab);
        columnView->setObjectName(QString::fromUtf8("columnView"));

        verticalLayout->addWidget(columnView);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        cmdAddjob = new QPushButton(tab);
        cmdAddjob->setObjectName(QString::fromUtf8("cmdAddjob"));

        horizontalLayout->addWidget(cmdAddjob);

        cmddeljob = new QPushButton(tab);
        cmddeljob->setObjectName(QString::fromUtf8("cmddeljob"));

        horizontalLayout->addWidget(cmddeljob);

        cmdProperties = new QPushButton(tab);
        cmdProperties->setObjectName(QString::fromUtf8("cmdProperties"));

        horizontalLayout->addWidget(cmdProperties);


        verticalLayout->addLayout(horizontalLayout);

        cmdjobRun = new QPushButton(tab);
        cmdjobRun->setObjectName(QString::fromUtf8("cmdjobRun"));

        verticalLayout->addWidget(cmdjobRun);


        verticalLayout_2->addLayout(verticalLayout);

        tabWidget->addTab(tab, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        tabWidget->addTab(tab_3, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        tabWidget->addTab(tab_2, QString());

        gridLayout->addWidget(tabWidget, 0, 0, 1, 1);

        frmMainClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(frmMainClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 714, 21));
        menu_File = new QMenu(menuBar);
        menu_File->setObjectName(QString::fromUtf8("menu_File"));
        frmMainClass->setMenuBar(menuBar);
        statusBar = new QStatusBar(frmMainClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        frmMainClass->setStatusBar(statusBar);

        menuBar->addAction(menu_File->menuAction());
        menu_File->addAction(action_New);
        menu_File->addAction(action_Open);
        menu_File->addAction(action_Save);
        menu_File->addAction(actionSave_Output);
        menu_File->addSeparator();
        menu_File->addAction(action_Properties);
        menu_File->addSeparator();
        menu_File->addAction(actionE_xit);

        retranslateUi(frmMainClass);
        QObject::connect(actionE_xit, SIGNAL(triggered()), frmMainClass, SLOT(close()));
        QObject::connect(cmdjobRun, SIGNAL(clicked()), frmMainClass, SLOT(jobRun()));
        QObject::connect(cmdAddjob, SIGNAL(clicked()), frmMainClass, SLOT(jobItemAdd()));
        QObject::connect(cmdProperties, SIGNAL(clicked()), frmMainClass, SLOT(jobItemModify()));
        QObject::connect(cmddeljob, SIGNAL(clicked()), frmMainClass, SLOT(jobItemDelete()));
        QObject::connect(action_New, SIGNAL(triggered()), frmMainClass, SLOT(jobNew()));
        QObject::connect(action_Open, SIGNAL(triggered()), frmMainClass, SLOT(jobOpen()));
        QObject::connect(action_Save, SIGNAL(triggered()), frmMainClass, SLOT(jobSave()));
        QObject::connect(actionSave_Output, SIGNAL(triggered()), frmMainClass, SLOT(jobSaveOutput()));

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(frmMainClass);
    } // setupUi

    void retranslateUi(QMainWindow *frmMainClass)
    {
        frmMainClass->setWindowTitle(QApplication::translate("frmMainClass", "Mie Scattering Calculator", 0, QApplication::UnicodeUTF8));
        action_New->setText(QApplication::translate("frmMainClass", "&New Job", 0, QApplication::UnicodeUTF8));
        action_Open->setText(QApplication::translate("frmMainClass", "&Open Job", 0, QApplication::UnicodeUTF8));
        action_Save->setText(QApplication::translate("frmMainClass", "&Save Job", 0, QApplication::UnicodeUTF8));
        actionE_xit->setText(QApplication::translate("frmMainClass", "E&xit", 0, QApplication::UnicodeUTF8));
        actionWn_Calculation->setText(QApplication::translate("frmMainClass", "Wn Calculation...", 0, QApplication::UnicodeUTF8));
        actionTau_Calculation->setText(QApplication::translate("frmMainClass", "tau Calculation...", 0, QApplication::UnicodeUTF8));
        actionPi_Calculation->setText(QApplication::translate("frmMainClass", "pi Calculation...", 0, QApplication::UnicodeUTF8));
        actionA_b_Calculation->setText(QApplication::translate("frmMainClass", "a, b Calculation...", 0, QApplication::UnicodeUTF8));
        actionExtinction->setText(QApplication::translate("frmMainClass", "Extinction", 0, QApplication::UnicodeUTF8));
        actionAbsorption->setText(QApplication::translate("frmMainClass", "Absorption", 0, QApplication::UnicodeUTF8));
        actionScattering->setText(QApplication::translate("frmMainClass", "Scattering", 0, QApplication::UnicodeUTF8));
        actionSave_Output->setText(QApplication::translate("frmMainClass", "Save &Output", 0, QApplication::UnicodeUTF8));
        action_Properties->setText(QApplication::translate("frmMainClass", "&Properties", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("frmMainClass", "Job Commands:", 0, QApplication::UnicodeUTF8));
        cmdAddjob->setText(QApplication::translate("frmMainClass", "&Add", 0, QApplication::UnicodeUTF8));
        cmddeljob->setText(QApplication::translate("frmMainClass", "&Delete", 0, QApplication::UnicodeUTF8));
        cmdProperties->setText(QApplication::translate("frmMainClass", "&Properties", 0, QApplication::UnicodeUTF8));
        cmdjobRun->setText(QApplication::translate("frmMainClass", "&Run Job", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("frmMainClass", "Job Options", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("frmMainClass", "Job Output", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("frmMainClass", "Plotting", 0, QApplication::UnicodeUTF8));
        menu_File->setTitle(QApplication::translate("frmMainClass", "&File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class frmMainClass: public Ui_frmMainClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FRMMAIN_H
