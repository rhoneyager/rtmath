/********************************************************************************
** Form generated from reading UI file 'frmmain.ui'
**
** Created: Mon Mar 21 14:22:39 2011
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
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QStatusBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_frmMain
{
public:
    QAction *action_New_Run;
    QAction *action_Open_Run;
    QAction *action_Save_Run;
    QAction *actionE_xit;
    QAction *action_New_Model;
    QAction *action_Open_Model;
    QAction *action_Save_Model;
    QAction *action_About_rtmath;
    QAction *action_atmos_New;
    QAction *action_atmos_Open;
    QAction *action_atmos_Save;
    QAction *action_Layers;
    QAction *action_New_2;
    QAction *action_Open_2;
    QAction *action_Save_2;
    QAction *action_Aerosols;
    QAction *action_Clouds;
    QAction *action_Overrides;
    QAction *actionModel_Selection;
    QAction *actionSet_Lighting_Sources;
    QAction *action_Bandpass;
    QAction *action_High_Pass;
    QAction *action_Low_Pass;
    QAction *action_Notch;
    QAction *action_Custom;
    QAction *action_None;
    QAction *action_custom;
    QAction *action_model_Properties;
    QWidget *centralWidget;
    QLabel *lblRev;
    QMenuBar *menuBar;
    QMenu *menu_File;
    QMenu *menu_Model;
    QMenu *menu_Atmosphere;
    QMenu *menu_Base_Atmosphere;
    QMenu *menu_Overlays;
    QMenu *menu_Surface;
    QMenu *menu_Lighting;
    QMenu *menu_Filters;
    QMenu *menu_Define_Filter;
    QMenu *menu_Select_Filter;
    QMenu *menu_Help;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *frmMain)
    {
        if (frmMain->objectName().isEmpty())
            frmMain->setObjectName(QString::fromUtf8("frmMain"));
        frmMain->resize(600, 600);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/rtlogo"), QSize(), QIcon::Normal, QIcon::Off);
        frmMain->setWindowIcon(icon);
        action_New_Run = new QAction(frmMain);
        action_New_Run->setObjectName(QString::fromUtf8("action_New_Run"));
        action_Open_Run = new QAction(frmMain);
        action_Open_Run->setObjectName(QString::fromUtf8("action_Open_Run"));
        action_Save_Run = new QAction(frmMain);
        action_Save_Run->setObjectName(QString::fromUtf8("action_Save_Run"));
        actionE_xit = new QAction(frmMain);
        actionE_xit->setObjectName(QString::fromUtf8("actionE_xit"));
        action_New_Model = new QAction(frmMain);
        action_New_Model->setObjectName(QString::fromUtf8("action_New_Model"));
        action_Open_Model = new QAction(frmMain);
        action_Open_Model->setObjectName(QString::fromUtf8("action_Open_Model"));
        action_Save_Model = new QAction(frmMain);
        action_Save_Model->setObjectName(QString::fromUtf8("action_Save_Model"));
        action_About_rtmath = new QAction(frmMain);
        action_About_rtmath->setObjectName(QString::fromUtf8("action_About_rtmath"));
        action_atmos_New = new QAction(frmMain);
        action_atmos_New->setObjectName(QString::fromUtf8("action_atmos_New"));
        action_atmos_Open = new QAction(frmMain);
        action_atmos_Open->setObjectName(QString::fromUtf8("action_atmos_Open"));
        action_atmos_Save = new QAction(frmMain);
        action_atmos_Save->setObjectName(QString::fromUtf8("action_atmos_Save"));
        action_Layers = new QAction(frmMain);
        action_Layers->setObjectName(QString::fromUtf8("action_Layers"));
        action_New_2 = new QAction(frmMain);
        action_New_2->setObjectName(QString::fromUtf8("action_New_2"));
        action_Open_2 = new QAction(frmMain);
        action_Open_2->setObjectName(QString::fromUtf8("action_Open_2"));
        action_Save_2 = new QAction(frmMain);
        action_Save_2->setObjectName(QString::fromUtf8("action_Save_2"));
        action_Aerosols = new QAction(frmMain);
        action_Aerosols->setObjectName(QString::fromUtf8("action_Aerosols"));
        action_Clouds = new QAction(frmMain);
        action_Clouds->setObjectName(QString::fromUtf8("action_Clouds"));
        action_Overrides = new QAction(frmMain);
        action_Overrides->setObjectName(QString::fromUtf8("action_Overrides"));
        actionModel_Selection = new QAction(frmMain);
        actionModel_Selection->setObjectName(QString::fromUtf8("actionModel_Selection"));
        actionSet_Lighting_Sources = new QAction(frmMain);
        actionSet_Lighting_Sources->setObjectName(QString::fromUtf8("actionSet_Lighting_Sources"));
        action_Bandpass = new QAction(frmMain);
        action_Bandpass->setObjectName(QString::fromUtf8("action_Bandpass"));
        action_High_Pass = new QAction(frmMain);
        action_High_Pass->setObjectName(QString::fromUtf8("action_High_Pass"));
        action_Low_Pass = new QAction(frmMain);
        action_Low_Pass->setObjectName(QString::fromUtf8("action_Low_Pass"));
        action_Notch = new QAction(frmMain);
        action_Notch->setObjectName(QString::fromUtf8("action_Notch"));
        action_Custom = new QAction(frmMain);
        action_Custom->setObjectName(QString::fromUtf8("action_Custom"));
        action_None = new QAction(frmMain);
        action_None->setObjectName(QString::fromUtf8("action_None"));
        action_custom = new QAction(frmMain);
        action_custom->setObjectName(QString::fromUtf8("action_custom"));
        action_model_Properties = new QAction(frmMain);
        action_model_Properties->setObjectName(QString::fromUtf8("action_model_Properties"));
        centralWidget = new QWidget(frmMain);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        lblRev = new QLabel(centralWidget);
        lblRev->setObjectName(QString::fromUtf8("lblRev"));
        lblRev->setGeometry(QRect(170, 270, 46, 13));
        frmMain->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(frmMain);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 600, 21));
        menu_File = new QMenu(menuBar);
        menu_File->setObjectName(QString::fromUtf8("menu_File"));
        menu_Model = new QMenu(menuBar);
        menu_Model->setObjectName(QString::fromUtf8("menu_Model"));
        menu_Atmosphere = new QMenu(menuBar);
        menu_Atmosphere->setObjectName(QString::fromUtf8("menu_Atmosphere"));
        menu_Base_Atmosphere = new QMenu(menu_Atmosphere);
        menu_Base_Atmosphere->setObjectName(QString::fromUtf8("menu_Base_Atmosphere"));
        menu_Overlays = new QMenu(menu_Atmosphere);
        menu_Overlays->setObjectName(QString::fromUtf8("menu_Overlays"));
        menu_Surface = new QMenu(menuBar);
        menu_Surface->setObjectName(QString::fromUtf8("menu_Surface"));
        menu_Lighting = new QMenu(menuBar);
        menu_Lighting->setObjectName(QString::fromUtf8("menu_Lighting"));
        menu_Filters = new QMenu(menuBar);
        menu_Filters->setObjectName(QString::fromUtf8("menu_Filters"));
        menu_Define_Filter = new QMenu(menu_Filters);
        menu_Define_Filter->setObjectName(QString::fromUtf8("menu_Define_Filter"));
        menu_Select_Filter = new QMenu(menu_Filters);
        menu_Select_Filter->setObjectName(QString::fromUtf8("menu_Select_Filter"));
        menu_Help = new QMenu(menuBar);
        menu_Help->setObjectName(QString::fromUtf8("menu_Help"));
        frmMain->setMenuBar(menuBar);
        statusBar = new QStatusBar(frmMain);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        frmMain->setStatusBar(statusBar);

        menuBar->addAction(menu_File->menuAction());
        menuBar->addAction(menu_Model->menuAction());
        menuBar->addAction(menu_Atmosphere->menuAction());
        menuBar->addAction(menu_Surface->menuAction());
        menuBar->addAction(menu_Lighting->menuAction());
        menuBar->addAction(menu_Filters->menuAction());
        menuBar->addAction(menu_Help->menuAction());
        menu_File->addAction(action_New_Run);
        menu_File->addAction(action_Open_Run);
        menu_File->addAction(action_Save_Run);
        menu_File->addSeparator();
        menu_File->addAction(actionE_xit);
        menu_Model->addAction(action_New_Model);
        menu_Model->addAction(action_Open_Model);
        menu_Model->addAction(action_Save_Model);
        menu_Model->addAction(action_model_Properties);
        menu_Atmosphere->addAction(menu_Base_Atmosphere->menuAction());
        menu_Atmosphere->addAction(menu_Overlays->menuAction());
        menu_Base_Atmosphere->addAction(action_atmos_New);
        menu_Base_Atmosphere->addAction(action_atmos_Open);
        menu_Base_Atmosphere->addAction(action_atmos_Save);
        menu_Base_Atmosphere->addSeparator();
        menu_Base_Atmosphere->addAction(action_Layers);
        menu_Overlays->addAction(action_New_2);
        menu_Overlays->addAction(action_Open_2);
        menu_Overlays->addAction(action_Save_2);
        menu_Overlays->addSeparator();
        menu_Overlays->addAction(action_Aerosols);
        menu_Overlays->addAction(action_Clouds);
        menu_Overlays->addAction(action_Overrides);
        menu_Surface->addAction(actionModel_Selection);
        menu_Lighting->addAction(actionSet_Lighting_Sources);
        menu_Filters->addAction(menu_Define_Filter->menuAction());
        menu_Filters->addAction(menu_Select_Filter->menuAction());
        menu_Define_Filter->addAction(action_Bandpass);
        menu_Define_Filter->addAction(action_High_Pass);
        menu_Define_Filter->addAction(action_Low_Pass);
        menu_Define_Filter->addAction(action_Notch);
        menu_Define_Filter->addSeparator();
        menu_Define_Filter->addAction(action_Custom);
        menu_Select_Filter->addAction(action_None);
        menu_Select_Filter->addSeparator();
        menu_Select_Filter->addAction(action_custom);
        menu_Help->addAction(action_About_rtmath);

        retranslateUi(frmMain);
        QObject::connect(actionE_xit, SIGNAL(triggered()), frmMain, SLOT(close()));
        QObject::connect(action_model_Properties, SIGNAL(triggered()), frmMain, SLOT(model_props()));
        QObject::connect(action_About_rtmath, SIGNAL(triggered()), frmMain, SLOT(show_about()));
        QObject::connect(action_atmos_Open, SIGNAL(triggered()), frmMain, SLOT(atmos_open()));

        QMetaObject::connectSlotsByName(frmMain);
    } // setupUi

    void retranslateUi(QMainWindow *frmMain)
    {
        frmMain->setWindowTitle(QApplication::translate("frmMain", "rtmath", 0, QApplication::UnicodeUTF8));
        action_New_Run->setText(QApplication::translate("frmMain", "&New Run", 0, QApplication::UnicodeUTF8));
        action_Open_Run->setText(QApplication::translate("frmMain", "&Open Run", 0, QApplication::UnicodeUTF8));
        action_Save_Run->setText(QApplication::translate("frmMain", "&Save Results", 0, QApplication::UnicodeUTF8));
        actionE_xit->setText(QApplication::translate("frmMain", "E&xit", 0, QApplication::UnicodeUTF8));
        action_New_Model->setText(QApplication::translate("frmMain", "&New Model", 0, QApplication::UnicodeUTF8));
        action_Open_Model->setText(QApplication::translate("frmMain", "&Open Model", 0, QApplication::UnicodeUTF8));
        action_Save_Model->setText(QApplication::translate("frmMain", "&Save Model", 0, QApplication::UnicodeUTF8));
        action_About_rtmath->setText(QApplication::translate("frmMain", "&About rtmath", 0, QApplication::UnicodeUTF8));
        action_atmos_New->setText(QApplication::translate("frmMain", "&New", 0, QApplication::UnicodeUTF8));
        action_atmos_Open->setText(QApplication::translate("frmMain", "&Open", 0, QApplication::UnicodeUTF8));
        action_atmos_Save->setText(QApplication::translate("frmMain", "&Save", 0, QApplication::UnicodeUTF8));
        action_Layers->setText(QApplication::translate("frmMain", "&Layers...", 0, QApplication::UnicodeUTF8));
        action_New_2->setText(QApplication::translate("frmMain", "&New", 0, QApplication::UnicodeUTF8));
        action_Open_2->setText(QApplication::translate("frmMain", "&Open", 0, QApplication::UnicodeUTF8));
        action_Save_2->setText(QApplication::translate("frmMain", "&Save", 0, QApplication::UnicodeUTF8));
        action_Aerosols->setText(QApplication::translate("frmMain", "&Aerosols...", 0, QApplication::UnicodeUTF8));
        action_Clouds->setText(QApplication::translate("frmMain", "&Clouds...", 0, QApplication::UnicodeUTF8));
        action_Overrides->setText(QApplication::translate("frmMain", "&Overrides...", 0, QApplication::UnicodeUTF8));
        actionModel_Selection->setText(QApplication::translate("frmMain", "Model Selection", 0, QApplication::UnicodeUTF8));
        actionSet_Lighting_Sources->setText(QApplication::translate("frmMain", "Set Lighting Sources...", 0, QApplication::UnicodeUTF8));
        action_Bandpass->setText(QApplication::translate("frmMain", "&Bandpass...", 0, QApplication::UnicodeUTF8));
        action_High_Pass->setText(QApplication::translate("frmMain", "&High-Pass...", 0, QApplication::UnicodeUTF8));
        action_Low_Pass->setText(QApplication::translate("frmMain", "&Low-Pass...", 0, QApplication::UnicodeUTF8));
        action_Notch->setText(QApplication::translate("frmMain", "&Notch...", 0, QApplication::UnicodeUTF8));
        action_Custom->setText(QApplication::translate("frmMain", "&Custom...", 0, QApplication::UnicodeUTF8));
        action_None->setText(QApplication::translate("frmMain", "&None", 0, QApplication::UnicodeUTF8));
        action_custom->setText(QApplication::translate("frmMain", "(custom)", 0, QApplication::UnicodeUTF8));
        action_model_Properties->setText(QApplication::translate("frmMain", "&Properties...", 0, QApplication::UnicodeUTF8));
        lblRev->setText(QApplication::translate("frmMain", "Rev", 0, QApplication::UnicodeUTF8));
        menu_File->setTitle(QApplication::translate("frmMain", "&File", 0, QApplication::UnicodeUTF8));
        menu_Model->setTitle(QApplication::translate("frmMain", "&Model", 0, QApplication::UnicodeUTF8));
        menu_Atmosphere->setTitle(QApplication::translate("frmMain", "&Atmosphere", 0, QApplication::UnicodeUTF8));
        menu_Base_Atmosphere->setTitle(QApplication::translate("frmMain", "&Base Atmosphere", 0, QApplication::UnicodeUTF8));
        menu_Overlays->setTitle(QApplication::translate("frmMain", "&Overlays", 0, QApplication::UnicodeUTF8));
        menu_Surface->setTitle(QApplication::translate("frmMain", "&Surface", 0, QApplication::UnicodeUTF8));
        menu_Lighting->setTitle(QApplication::translate("frmMain", "&Lighting", 0, QApplication::UnicodeUTF8));
        menu_Filters->setTitle(QApplication::translate("frmMain", "Fil&ters", 0, QApplication::UnicodeUTF8));
        menu_Define_Filter->setTitle(QApplication::translate("frmMain", "&Define Filter", 0, QApplication::UnicodeUTF8));
        menu_Select_Filter->setTitle(QApplication::translate("frmMain", "&Select Filter", 0, QApplication::UnicodeUTF8));
        menu_Help->setTitle(QApplication::translate("frmMain", "&Help", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class frmMain: public Ui_frmMain {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FRMMAIN_H
