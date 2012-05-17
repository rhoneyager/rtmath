/********************************************************************************
** Form generated from reading UI file 'frmmain.ui'
**
** Created: Tue May 15 21:01:44 2012
**      by: Qt User Interface Compiler version 4.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FRMMAIN_H
#define UI_FRMMAIN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QFormLayout>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QListWidget>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPlainTextEdit>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QTableWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_frmMainClass
{
public:
    QAction *actionE_xit;
    QAction *action_Open;
    QAction *action_Save;
    QAction *action_New;
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout;
    QTabWidget *tabWidget;
    QWidget *tabGeneral;
    QFormLayout *formLayout;
    QLabel *label_2;
    QLineEdit *txtRunName;
    QLabel *label_3;
    QPlainTextEdit *txtDescription;
    QLabel *label;
    QHBoxLayout *horizontalLayout_2;
    QLineEdit *txtBaseFile;
    QPushButton *cmdSelectBaseFile;
    QLabel *label_6;
    QHBoxLayout *horizontalLayout_3;
    QLineEdit *txtOutLocation;
    QPushButton *cmdSelectOutputLocation;
    QLabel *label_10;
    QComboBox *cmbDdver;
    QLabel *label_9;
    QCheckBox *chkGenIndivRunScripts;
    QLabel *label_8;
    QCheckBox *chkGenMassRunScript;
    QLabel *label_7;
    QCheckBox *chkGenShapeStats;
    QLabel *label_4;
    QCheckBox *chkDatabaseRegister;
    QLabel *label_5;
    QPlainTextEdit *txtSummary;
    QLabel *label_24;
    QCheckBox *chkCompress;
    QLabel *label_25;
    QHBoxLayout *horizontalLayout_8;
    QCheckBox *chkDoExport;
    QLineEdit *txtExportDir;
    QPushButton *cmdSelectExportDir;
    QWidget *tab_2;
    QFormLayout *formLayout_2;
    QLabel *label_11;
    QCheckBox *chkDoTorques;
    QLabel *label_12;
    QComboBox *cmbSolnMeth;
    QLabel *label_13;
    QComboBox *cmbFFTsolver;
    QLabel *label_14;
    QComboBox *cmbCALPHA;
    QLabel *label_15;
    QComboBox *cmbBinning;
    QLabel *label_16;
    QHBoxLayout *horizontalLayout_4;
    QLineEdit *txtImem1;
    QLineEdit *txtImem2;
    QLineEdit *txtImem3;
    QLabel *label_17;
    QCheckBox *chkNearfield;
    QLabel *label_18;
    QHBoxLayout *horizontalLayout_7;
    QLineEdit *txtNear1;
    QLineEdit *txtNear2;
    QLineEdit *txtNear3;
    QLineEdit *txtNear4;
    QLineEdit *txtNear5;
    QLineEdit *txtNear6;
    QLabel *label_19;
    QLabel *label_20;
    QLabel *label_22;
    QLabel *label_21;
    QLabel *label_23;
    QLineEdit *txtMaxTol;
    QLineEdit *txtMaxIter;
    QLineEdit *txtGamma;
    QLineEdit *txtETASCA;
    QLineEdit *txtNAMBIENT;
    QWidget *tab;
    QFormLayout *formLayout_3;
    QLabel *label_26;
    QComboBox *comboBox;
    QLabel *label_27;
    QLineEdit *lineEdit_2;
    QLabel *label_28;
    QHBoxLayout *horizontalLayout_9;
    QLineEdit *lineEdit_5;
    QLineEdit *lineEdit_4;
    QLineEdit *lineEdit_3;
    QLabel *label_31;
    QTableWidget *tableWidget_3;
    QLabel *label_29;
    QDoubleSpinBox *doubleSpinBox;
    QHBoxLayout *horizontalLayout_10;
    QLineEdit *lineEdit_7;
    QComboBox *comboBox_2;
    QLabel *label_30;
    QWidget *tab_3;
    QFormLayout *formLayout_4;
    QLabel *label_32;
    QHBoxLayout *horizontalLayout_11;
    QPushButton *pushButton_5;
    QPushButton *pushButton_2;
    QPushButton *pushButton;
    QListWidget *listWidget;
    QListWidget *listWidget_2;
    QLabel *label_33;
    QHBoxLayout *horizontalLayout_12;
    QPushButton *pushButton_4;
    QPushButton *pushButton_3;
    QPushButton *pushButton_6;
    QWidget *tab_4;
    QGridLayout *gridLayout;
    QLabel *label_34;
    QTableWidget *tableWidget;
    QLabel *label_36;
    QHBoxLayout *horizontalLayout_13;
    QCheckBox *checkBox;
    QLineEdit *lineEdit;
    QLabel *label_35;
    QTableWidget *tableWidget_2;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer;
    QPushButton *cmdGenerate;
    QMenuBar *menuBar;
    QMenu *menu_File;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *frmMainClass)
    {
        if (frmMainClass->objectName().isEmpty())
            frmMainClass->setObjectName(QString::fromUtf8("frmMainClass"));
        frmMainClass->resize(815, 614);
        actionE_xit = new QAction(frmMainClass);
        actionE_xit->setObjectName(QString::fromUtf8("actionE_xit"));
        action_Open = new QAction(frmMainClass);
        action_Open->setObjectName(QString::fromUtf8("action_Open"));
        action_Save = new QAction(frmMainClass);
        action_Save->setObjectName(QString::fromUtf8("action_Save"));
        action_New = new QAction(frmMainClass);
        action_New->setObjectName(QString::fromUtf8("action_New"));
        centralWidget = new QWidget(frmMainClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        verticalLayout = new QVBoxLayout(centralWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabGeneral = new QWidget();
        tabGeneral->setObjectName(QString::fromUtf8("tabGeneral"));
        formLayout = new QFormLayout(tabGeneral);
        formLayout->setSpacing(6);
        formLayout->setContentsMargins(11, 11, 11, 11);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        formLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        label_2 = new QLabel(tabGeneral);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        formLayout->setWidget(0, QFormLayout::LabelRole, label_2);

        txtRunName = new QLineEdit(tabGeneral);
        txtRunName->setObjectName(QString::fromUtf8("txtRunName"));

        formLayout->setWidget(0, QFormLayout::FieldRole, txtRunName);

        label_3 = new QLabel(tabGeneral);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        formLayout->setWidget(1, QFormLayout::LabelRole, label_3);

        txtDescription = new QPlainTextEdit(tabGeneral);
        txtDescription->setObjectName(QString::fromUtf8("txtDescription"));

        formLayout->setWidget(1, QFormLayout::FieldRole, txtDescription);

        label = new QLabel(tabGeneral);
        label->setObjectName(QString::fromUtf8("label"));
        label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        formLayout->setWidget(3, QFormLayout::LabelRole, label);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        txtBaseFile = new QLineEdit(tabGeneral);
        txtBaseFile->setObjectName(QString::fromUtf8("txtBaseFile"));

        horizontalLayout_2->addWidget(txtBaseFile);

        cmdSelectBaseFile = new QPushButton(tabGeneral);
        cmdSelectBaseFile->setObjectName(QString::fromUtf8("cmdSelectBaseFile"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(cmdSelectBaseFile->sizePolicy().hasHeightForWidth());
        cmdSelectBaseFile->setSizePolicy(sizePolicy);

        horizontalLayout_2->addWidget(cmdSelectBaseFile);


        formLayout->setLayout(3, QFormLayout::FieldRole, horizontalLayout_2);

        label_6 = new QLabel(tabGeneral);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        formLayout->setWidget(5, QFormLayout::LabelRole, label_6);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        txtOutLocation = new QLineEdit(tabGeneral);
        txtOutLocation->setObjectName(QString::fromUtf8("txtOutLocation"));

        horizontalLayout_3->addWidget(txtOutLocation);

        cmdSelectOutputLocation = new QPushButton(tabGeneral);
        cmdSelectOutputLocation->setObjectName(QString::fromUtf8("cmdSelectOutputLocation"));

        horizontalLayout_3->addWidget(cmdSelectOutputLocation);


        formLayout->setLayout(5, QFormLayout::FieldRole, horizontalLayout_3);

        label_10 = new QLabel(tabGeneral);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        formLayout->setWidget(6, QFormLayout::LabelRole, label_10);

        cmbDdver = new QComboBox(tabGeneral);
        cmbDdver->setObjectName(QString::fromUtf8("cmbDdver"));

        formLayout->setWidget(6, QFormLayout::FieldRole, cmbDdver);

        label_9 = new QLabel(tabGeneral);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        formLayout->setWidget(7, QFormLayout::LabelRole, label_9);

        chkGenIndivRunScripts = new QCheckBox(tabGeneral);
        chkGenIndivRunScripts->setObjectName(QString::fromUtf8("chkGenIndivRunScripts"));
        chkGenIndivRunScripts->setChecked(true);

        formLayout->setWidget(7, QFormLayout::FieldRole, chkGenIndivRunScripts);

        label_8 = new QLabel(tabGeneral);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        formLayout->setWidget(8, QFormLayout::LabelRole, label_8);

        chkGenMassRunScript = new QCheckBox(tabGeneral);
        chkGenMassRunScript->setObjectName(QString::fromUtf8("chkGenMassRunScript"));
        chkGenMassRunScript->setChecked(true);

        formLayout->setWidget(8, QFormLayout::FieldRole, chkGenMassRunScript);

        label_7 = new QLabel(tabGeneral);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        formLayout->setWidget(9, QFormLayout::LabelRole, label_7);

        chkGenShapeStats = new QCheckBox(tabGeneral);
        chkGenShapeStats->setObjectName(QString::fromUtf8("chkGenShapeStats"));
        chkGenShapeStats->setEnabled(false);
        chkGenShapeStats->setChecked(false);

        formLayout->setWidget(9, QFormLayout::FieldRole, chkGenShapeStats);

        label_4 = new QLabel(tabGeneral);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        formLayout->setWidget(10, QFormLayout::LabelRole, label_4);

        chkDatabaseRegister = new QCheckBox(tabGeneral);
        chkDatabaseRegister->setObjectName(QString::fromUtf8("chkDatabaseRegister"));
        chkDatabaseRegister->setEnabled(false);

        formLayout->setWidget(10, QFormLayout::FieldRole, chkDatabaseRegister);

        label_5 = new QLabel(tabGeneral);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        formLayout->setWidget(17, QFormLayout::LabelRole, label_5);

        txtSummary = new QPlainTextEdit(tabGeneral);
        txtSummary->setObjectName(QString::fromUtf8("txtSummary"));
        txtSummary->setReadOnly(true);

        formLayout->setWidget(17, QFormLayout::FieldRole, txtSummary);

        label_24 = new QLabel(tabGeneral);
        label_24->setObjectName(QString::fromUtf8("label_24"));

        formLayout->setWidget(14, QFormLayout::LabelRole, label_24);

        chkCompress = new QCheckBox(tabGeneral);
        chkCompress->setObjectName(QString::fromUtf8("chkCompress"));
        chkCompress->setChecked(true);

        formLayout->setWidget(14, QFormLayout::FieldRole, chkCompress);

        label_25 = new QLabel(tabGeneral);
        label_25->setObjectName(QString::fromUtf8("label_25"));

        formLayout->setWidget(15, QFormLayout::LabelRole, label_25);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        chkDoExport = new QCheckBox(tabGeneral);
        chkDoExport->setObjectName(QString::fromUtf8("chkDoExport"));

        horizontalLayout_8->addWidget(chkDoExport);

        txtExportDir = new QLineEdit(tabGeneral);
        txtExportDir->setObjectName(QString::fromUtf8("txtExportDir"));
        txtExportDir->setEnabled(false);

        horizontalLayout_8->addWidget(txtExportDir);

        cmdSelectExportDir = new QPushButton(tabGeneral);
        cmdSelectExportDir->setObjectName(QString::fromUtf8("cmdSelectExportDir"));
        cmdSelectExportDir->setEnabled(false);

        horizontalLayout_8->addWidget(cmdSelectExportDir);


        formLayout->setLayout(15, QFormLayout::FieldRole, horizontalLayout_8);

        tabWidget->addTab(tabGeneral, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        formLayout_2 = new QFormLayout(tab_2);
        formLayout_2->setSpacing(6);
        formLayout_2->setContentsMargins(11, 11, 11, 11);
        formLayout_2->setObjectName(QString::fromUtf8("formLayout_2"));
        formLayout_2->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        label_11 = new QLabel(tab_2);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        formLayout_2->setWidget(0, QFormLayout::LabelRole, label_11);

        chkDoTorques = new QCheckBox(tab_2);
        chkDoTorques->setObjectName(QString::fromUtf8("chkDoTorques"));

        formLayout_2->setWidget(0, QFormLayout::FieldRole, chkDoTorques);

        label_12 = new QLabel(tab_2);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        formLayout_2->setWidget(1, QFormLayout::LabelRole, label_12);

        cmbSolnMeth = new QComboBox(tab_2);
        cmbSolnMeth->setObjectName(QString::fromUtf8("cmbSolnMeth"));

        formLayout_2->setWidget(1, QFormLayout::FieldRole, cmbSolnMeth);

        label_13 = new QLabel(tab_2);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        formLayout_2->setWidget(2, QFormLayout::LabelRole, label_13);

        cmbFFTsolver = new QComboBox(tab_2);
        cmbFFTsolver->setObjectName(QString::fromUtf8("cmbFFTsolver"));

        formLayout_2->setWidget(2, QFormLayout::FieldRole, cmbFFTsolver);

        label_14 = new QLabel(tab_2);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        formLayout_2->setWidget(3, QFormLayout::LabelRole, label_14);

        cmbCALPHA = new QComboBox(tab_2);
        cmbCALPHA->setObjectName(QString::fromUtf8("cmbCALPHA"));

        formLayout_2->setWidget(3, QFormLayout::FieldRole, cmbCALPHA);

        label_15 = new QLabel(tab_2);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        formLayout_2->setWidget(4, QFormLayout::LabelRole, label_15);

        cmbBinning = new QComboBox(tab_2);
        cmbBinning->setObjectName(QString::fromUtf8("cmbBinning"));

        formLayout_2->setWidget(4, QFormLayout::FieldRole, cmbBinning);

        label_16 = new QLabel(tab_2);
        label_16->setObjectName(QString::fromUtf8("label_16"));

        formLayout_2->setWidget(5, QFormLayout::LabelRole, label_16);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        txtImem1 = new QLineEdit(tab_2);
        txtImem1->setObjectName(QString::fromUtf8("txtImem1"));

        horizontalLayout_4->addWidget(txtImem1);

        txtImem2 = new QLineEdit(tab_2);
        txtImem2->setObjectName(QString::fromUtf8("txtImem2"));

        horizontalLayout_4->addWidget(txtImem2);

        txtImem3 = new QLineEdit(tab_2);
        txtImem3->setObjectName(QString::fromUtf8("txtImem3"));

        horizontalLayout_4->addWidget(txtImem3);


        formLayout_2->setLayout(5, QFormLayout::FieldRole, horizontalLayout_4);

        label_17 = new QLabel(tab_2);
        label_17->setObjectName(QString::fromUtf8("label_17"));

        formLayout_2->setWidget(6, QFormLayout::LabelRole, label_17);

        chkNearfield = new QCheckBox(tab_2);
        chkNearfield->setObjectName(QString::fromUtf8("chkNearfield"));

        formLayout_2->setWidget(6, QFormLayout::FieldRole, chkNearfield);

        label_18 = new QLabel(tab_2);
        label_18->setObjectName(QString::fromUtf8("label_18"));

        formLayout_2->setWidget(7, QFormLayout::LabelRole, label_18);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        txtNear1 = new QLineEdit(tab_2);
        txtNear1->setObjectName(QString::fromUtf8("txtNear1"));

        horizontalLayout_7->addWidget(txtNear1);

        txtNear2 = new QLineEdit(tab_2);
        txtNear2->setObjectName(QString::fromUtf8("txtNear2"));

        horizontalLayout_7->addWidget(txtNear2);

        txtNear3 = new QLineEdit(tab_2);
        txtNear3->setObjectName(QString::fromUtf8("txtNear3"));

        horizontalLayout_7->addWidget(txtNear3);

        txtNear4 = new QLineEdit(tab_2);
        txtNear4->setObjectName(QString::fromUtf8("txtNear4"));

        horizontalLayout_7->addWidget(txtNear4);

        txtNear5 = new QLineEdit(tab_2);
        txtNear5->setObjectName(QString::fromUtf8("txtNear5"));
        txtNear5->setReadOnly(false);

        horizontalLayout_7->addWidget(txtNear5);

        txtNear6 = new QLineEdit(tab_2);
        txtNear6->setObjectName(QString::fromUtf8("txtNear6"));
        txtNear6->setReadOnly(false);

        horizontalLayout_7->addWidget(txtNear6);


        formLayout_2->setLayout(7, QFormLayout::FieldRole, horizontalLayout_7);

        label_19 = new QLabel(tab_2);
        label_19->setObjectName(QString::fromUtf8("label_19"));

        formLayout_2->setWidget(8, QFormLayout::LabelRole, label_19);

        label_20 = new QLabel(tab_2);
        label_20->setObjectName(QString::fromUtf8("label_20"));

        formLayout_2->setWidget(9, QFormLayout::LabelRole, label_20);

        label_22 = new QLabel(tab_2);
        label_22->setObjectName(QString::fromUtf8("label_22"));

        formLayout_2->setWidget(10, QFormLayout::LabelRole, label_22);

        label_21 = new QLabel(tab_2);
        label_21->setObjectName(QString::fromUtf8("label_21"));

        formLayout_2->setWidget(11, QFormLayout::LabelRole, label_21);

        label_23 = new QLabel(tab_2);
        label_23->setObjectName(QString::fromUtf8("label_23"));

        formLayout_2->setWidget(12, QFormLayout::LabelRole, label_23);

        txtMaxTol = new QLineEdit(tab_2);
        txtMaxTol->setObjectName(QString::fromUtf8("txtMaxTol"));

        formLayout_2->setWidget(8, QFormLayout::FieldRole, txtMaxTol);

        txtMaxIter = new QLineEdit(tab_2);
        txtMaxIter->setObjectName(QString::fromUtf8("txtMaxIter"));

        formLayout_2->setWidget(9, QFormLayout::FieldRole, txtMaxIter);

        txtGamma = new QLineEdit(tab_2);
        txtGamma->setObjectName(QString::fromUtf8("txtGamma"));

        formLayout_2->setWidget(10, QFormLayout::FieldRole, txtGamma);

        txtETASCA = new QLineEdit(tab_2);
        txtETASCA->setObjectName(QString::fromUtf8("txtETASCA"));

        formLayout_2->setWidget(11, QFormLayout::FieldRole, txtETASCA);

        txtNAMBIENT = new QLineEdit(tab_2);
        txtNAMBIENT->setObjectName(QString::fromUtf8("txtNAMBIENT"));

        formLayout_2->setWidget(12, QFormLayout::FieldRole, txtNAMBIENT);

        tabWidget->addTab(tab_2, QString());
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        formLayout_3 = new QFormLayout(tab);
        formLayout_3->setSpacing(6);
        formLayout_3->setContentsMargins(11, 11, 11, 11);
        formLayout_3->setObjectName(QString::fromUtf8("formLayout_3"));
        formLayout_3->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        label_26 = new QLabel(tab);
        label_26->setObjectName(QString::fromUtf8("label_26"));

        formLayout_3->setWidget(0, QFormLayout::LabelRole, label_26);

        comboBox = new QComboBox(tab);
        comboBox->setObjectName(QString::fromUtf8("comboBox"));

        formLayout_3->setWidget(0, QFormLayout::FieldRole, comboBox);

        label_27 = new QLabel(tab);
        label_27->setObjectName(QString::fromUtf8("label_27"));

        formLayout_3->setWidget(1, QFormLayout::LabelRole, label_27);

        lineEdit_2 = new QLineEdit(tab);
        lineEdit_2->setObjectName(QString::fromUtf8("lineEdit_2"));

        formLayout_3->setWidget(1, QFormLayout::FieldRole, lineEdit_2);

        label_28 = new QLabel(tab);
        label_28->setObjectName(QString::fromUtf8("label_28"));

        formLayout_3->setWidget(2, QFormLayout::LabelRole, label_28);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        lineEdit_5 = new QLineEdit(tab);
        lineEdit_5->setObjectName(QString::fromUtf8("lineEdit_5"));

        horizontalLayout_9->addWidget(lineEdit_5);

        lineEdit_4 = new QLineEdit(tab);
        lineEdit_4->setObjectName(QString::fromUtf8("lineEdit_4"));

        horizontalLayout_9->addWidget(lineEdit_4);

        lineEdit_3 = new QLineEdit(tab);
        lineEdit_3->setObjectName(QString::fromUtf8("lineEdit_3"));

        horizontalLayout_9->addWidget(lineEdit_3);


        formLayout_3->setLayout(2, QFormLayout::FieldRole, horizontalLayout_9);

        label_31 = new QLabel(tab);
        label_31->setObjectName(QString::fromUtf8("label_31"));

        formLayout_3->setWidget(12, QFormLayout::SpanningRole, label_31);

        tableWidget_3 = new QTableWidget(tab);
        if (tableWidget_3->columnCount() < 1)
            tableWidget_3->setColumnCount(1);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        tableWidget_3->setHorizontalHeaderItem(0, __qtablewidgetitem);
        if (tableWidget_3->rowCount() < 1)
            tableWidget_3->setRowCount(1);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        tableWidget_3->setVerticalHeaderItem(0, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        tableWidget_3->setItem(0, 0, __qtablewidgetitem2);
        tableWidget_3->setObjectName(QString::fromUtf8("tableWidget_3"));
        tableWidget_3->setContextMenuPolicy(Qt::CustomContextMenu);
        tableWidget_3->setDragEnabled(false);
        tableWidget_3->setRowCount(1);
        tableWidget_3->setColumnCount(1);

        formLayout_3->setWidget(13, QFormLayout::FieldRole, tableWidget_3);

        label_29 = new QLabel(tab);
        label_29->setObjectName(QString::fromUtf8("label_29"));

        formLayout_3->setWidget(3, QFormLayout::LabelRole, label_29);

        doubleSpinBox = new QDoubleSpinBox(tab);
        doubleSpinBox->setObjectName(QString::fromUtf8("doubleSpinBox"));
        doubleSpinBox->setValue(5);

        formLayout_3->setWidget(3, QFormLayout::FieldRole, doubleSpinBox);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        lineEdit_7 = new QLineEdit(tab);
        lineEdit_7->setObjectName(QString::fromUtf8("lineEdit_7"));

        horizontalLayout_10->addWidget(lineEdit_7);

        comboBox_2 = new QComboBox(tab);
        comboBox_2->setObjectName(QString::fromUtf8("comboBox_2"));

        horizontalLayout_10->addWidget(comboBox_2);


        formLayout_3->setLayout(4, QFormLayout::FieldRole, horizontalLayout_10);

        label_30 = new QLabel(tab);
        label_30->setObjectName(QString::fromUtf8("label_30"));

        formLayout_3->setWidget(4, QFormLayout::LabelRole, label_30);

        tabWidget->addTab(tab, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        formLayout_4 = new QFormLayout(tab_3);
        formLayout_4->setSpacing(6);
        formLayout_4->setContentsMargins(11, 11, 11, 11);
        formLayout_4->setObjectName(QString::fromUtf8("formLayout_4"));
        label_32 = new QLabel(tab_3);
        label_32->setObjectName(QString::fromUtf8("label_32"));

        formLayout_4->setWidget(0, QFormLayout::LabelRole, label_32);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setSpacing(6);
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        pushButton_5 = new QPushButton(tab_3);
        pushButton_5->setObjectName(QString::fromUtf8("pushButton_5"));

        horizontalLayout_11->addWidget(pushButton_5);

        pushButton_2 = new QPushButton(tab_3);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));

        horizontalLayout_11->addWidget(pushButton_2);

        pushButton = new QPushButton(tab_3);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        horizontalLayout_11->addWidget(pushButton);


        formLayout_4->setLayout(2, QFormLayout::FieldRole, horizontalLayout_11);

        listWidget = new QListWidget(tab_3);
        listWidget->setObjectName(QString::fromUtf8("listWidget"));

        formLayout_4->setWidget(0, QFormLayout::FieldRole, listWidget);

        listWidget_2 = new QListWidget(tab_3);
        listWidget_2->setObjectName(QString::fromUtf8("listWidget_2"));

        formLayout_4->setWidget(3, QFormLayout::FieldRole, listWidget_2);

        label_33 = new QLabel(tab_3);
        label_33->setObjectName(QString::fromUtf8("label_33"));

        formLayout_4->setWidget(3, QFormLayout::LabelRole, label_33);

        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setSpacing(6);
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
        pushButton_4 = new QPushButton(tab_3);
        pushButton_4->setObjectName(QString::fromUtf8("pushButton_4"));

        horizontalLayout_12->addWidget(pushButton_4);

        pushButton_3 = new QPushButton(tab_3);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));

        horizontalLayout_12->addWidget(pushButton_3);

        pushButton_6 = new QPushButton(tab_3);
        pushButton_6->setObjectName(QString::fromUtf8("pushButton_6"));

        horizontalLayout_12->addWidget(pushButton_6);


        formLayout_4->setLayout(4, QFormLayout::FieldRole, horizontalLayout_12);

        tabWidget->addTab(tab_3, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QString::fromUtf8("tab_4"));
        gridLayout = new QGridLayout(tab_4);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_34 = new QLabel(tab_4);
        label_34->setObjectName(QString::fromUtf8("label_34"));

        gridLayout->addWidget(label_34, 0, 0, 1, 1);

        tableWidget = new QTableWidget(tab_4);
        if (tableWidget->columnCount() < 3)
            tableWidget->setColumnCount(3);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        tableWidget->setHorizontalHeaderItem(0, __qtablewidgetitem3);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        tableWidget->setHorizontalHeaderItem(1, __qtablewidgetitem4);
        QTableWidgetItem *__qtablewidgetitem5 = new QTableWidgetItem();
        tableWidget->setHorizontalHeaderItem(2, __qtablewidgetitem5);
        if (tableWidget->rowCount() < 3)
            tableWidget->setRowCount(3);
        QTableWidgetItem *__qtablewidgetitem6 = new QTableWidgetItem();
        tableWidget->setVerticalHeaderItem(0, __qtablewidgetitem6);
        QTableWidgetItem *__qtablewidgetitem7 = new QTableWidgetItem();
        tableWidget->setVerticalHeaderItem(1, __qtablewidgetitem7);
        QTableWidgetItem *__qtablewidgetitem8 = new QTableWidgetItem();
        tableWidget->setVerticalHeaderItem(2, __qtablewidgetitem8);
        tableWidget->setObjectName(QString::fromUtf8("tableWidget"));
        tableWidget->setAcceptDrops(false);
        tableWidget->setDragEnabled(false);
        tableWidget->setDragDropMode(QAbstractItemView::NoDragDrop);
        tableWidget->setDefaultDropAction(Qt::IgnoreAction);
        tableWidget->setRowCount(3);
        tableWidget->setColumnCount(3);

        gridLayout->addWidget(tableWidget, 0, 2, 1, 1);

        label_36 = new QLabel(tab_4);
        label_36->setObjectName(QString::fromUtf8("label_36"));

        gridLayout->addWidget(label_36, 1, 0, 1, 2);

        horizontalLayout_13 = new QHBoxLayout();
        horizontalLayout_13->setSpacing(6);
        horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
        checkBox = new QCheckBox(tab_4);
        checkBox->setObjectName(QString::fromUtf8("checkBox"));

        horizontalLayout_13->addWidget(checkBox);

        lineEdit = new QLineEdit(tab_4);
        lineEdit->setObjectName(QString::fromUtf8("lineEdit"));

        horizontalLayout_13->addWidget(lineEdit);


        gridLayout->addLayout(horizontalLayout_13, 1, 2, 1, 1);

        label_35 = new QLabel(tab_4);
        label_35->setObjectName(QString::fromUtf8("label_35"));

        gridLayout->addWidget(label_35, 2, 0, 1, 1);

        tableWidget_2 = new QTableWidget(tab_4);
        if (tableWidget_2->columnCount() < 4)
            tableWidget_2->setColumnCount(4);
        QTableWidgetItem *__qtablewidgetitem9 = new QTableWidgetItem();
        tableWidget_2->setHorizontalHeaderItem(0, __qtablewidgetitem9);
        QTableWidgetItem *__qtablewidgetitem10 = new QTableWidgetItem();
        tableWidget_2->setHorizontalHeaderItem(1, __qtablewidgetitem10);
        QTableWidgetItem *__qtablewidgetitem11 = new QTableWidgetItem();
        tableWidget_2->setHorizontalHeaderItem(2, __qtablewidgetitem11);
        QTableWidgetItem *__qtablewidgetitem12 = new QTableWidgetItem();
        tableWidget_2->setHorizontalHeaderItem(3, __qtablewidgetitem12);
        if (tableWidget_2->rowCount() < 2)
            tableWidget_2->setRowCount(2);
        QTableWidgetItem *__qtablewidgetitem13 = new QTableWidgetItem();
        tableWidget_2->setVerticalHeaderItem(0, __qtablewidgetitem13);
        QTableWidgetItem *__qtablewidgetitem14 = new QTableWidgetItem();
        tableWidget_2->setVerticalHeaderItem(1, __qtablewidgetitem14);
        QTableWidgetItem *__qtablewidgetitem15 = new QTableWidgetItem();
        __qtablewidgetitem15->setCheckState(Qt::Unchecked);
        tableWidget_2->setItem(0, 0, __qtablewidgetitem15);
        tableWidget_2->setObjectName(QString::fromUtf8("tableWidget_2"));

        gridLayout->addWidget(tableWidget_2, 2, 1, 1, 2);

        tabWidget->addTab(tab_4, QString());

        verticalLayout->addWidget(tabWidget);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        cmdGenerate = new QPushButton(centralWidget);
        cmdGenerate->setObjectName(QString::fromUtf8("cmdGenerate"));
        cmdGenerate->setEnabled(false);

        horizontalLayout->addWidget(cmdGenerate);


        verticalLayout->addLayout(horizontalLayout);

        frmMainClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(frmMainClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 815, 21));
        menu_File = new QMenu(menuBar);
        menu_File->setObjectName(QString::fromUtf8("menu_File"));
        frmMainClass->setMenuBar(menuBar);
        statusBar = new QStatusBar(frmMainClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        frmMainClass->setStatusBar(statusBar);

        menuBar->addAction(menu_File->menuAction());
        menu_File->addAction(action_New);
        menu_File->addSeparator();
        menu_File->addAction(action_Open);
        menu_File->addAction(action_Save);
        menu_File->addSeparator();
        menu_File->addAction(actionE_xit);

        retranslateUi(frmMainClass);
        QObject::connect(actionE_xit, SIGNAL(activated()), frmMainClass, SLOT(close()));
        QObject::connect(chkDoExport, SIGNAL(stateChanged(int)), frmMainClass, SLOT(showMaximized()));

        tabWidget->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(frmMainClass);
    } // setupUi

    void retranslateUi(QMainWindow *frmMainClass)
    {
        frmMainClass->setWindowTitle(QApplication::translate("frmMainClass", "rtmath - DDSCAT Parameter Batch Generation Utility", 0, QApplication::UnicodeUTF8));
        actionE_xit->setText(QApplication::translate("frmMainClass", "E&xit", 0, QApplication::UnicodeUTF8));
        action_Open->setText(QApplication::translate("frmMainClass", "&Open", 0, QApplication::UnicodeUTF8));
        action_Save->setText(QApplication::translate("frmMainClass", "&Save", 0, QApplication::UnicodeUTF8));
        action_New->setText(QApplication::translate("frmMainClass", "&New", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("frmMainClass", "Run Name:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        txtRunName->setStatusTip(QApplication::translate("frmMainClass", "An easy-to-remember name for this set of runs", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        label_3->setText(QApplication::translate("frmMainClass", "Description:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        txtDescription->setStatusTip(QApplication::translate("frmMainClass", "A more detailed description of the runs", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        label->setText(QApplication::translate("frmMainClass", "Base ddscat.par file:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        txtBaseFile->setStatusTip(QApplication::translate("frmMainClass", "Defaults are loaded from this file. They are overridden in the next few screens.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
#ifndef QT_NO_TOOLTIP
        cmdSelectBaseFile->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        cmdSelectBaseFile->setText(QApplication::translate("frmMainClass", "...", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("frmMainClass", "Output Directory Tree Location:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        txtOutLocation->setStatusTip(QApplication::translate("frmMainClass", "The directory for storing all of the ddscat run information and the future directory tree.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        txtOutLocation->setPlaceholderText(QApplication::translate("frmMainClass", "./out", 0, QApplication::UnicodeUTF8));
        cmdSelectOutputLocation->setText(QApplication::translate("frmMainClass", "...", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("frmMainClass", "DDSCAT Version:", 0, QApplication::UnicodeUTF8));
        cmbDdver->clear();
        cmbDdver->insertItems(0, QStringList()
         << QApplication::translate("frmMainClass", "7.0 gcc", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmMainClass", "7.0 intel", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmMainClass", "7.2 gcc", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmMainClass", "7.2 intel", 0, QApplication::UnicodeUTF8)
        );
#ifndef QT_NO_STATUSTIP
        cmbDdver->setStatusTip(QApplication::translate("frmMainClass", "Specify ddscat version. 7.0 or 7.2? Intel or gcc?", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        label_9->setText(QApplication::translate("frmMainClass", "Generate Individual Run Scripts?", 0, QApplication::UnicodeUTF8));
        chkGenIndivRunScripts->setText(QString());
        label_8->setText(QApplication::translate("frmMainClass", "Generate Mass Run Script?", 0, QApplication::UnicodeUTF8));
        chkGenMassRunScript->setText(QString());
        label_7->setText(QApplication::translate("frmMainClass", "Generate Shape Statistics?", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        chkGenShapeStats->setStatusTip(QApplication::translate("frmMainClass", "TODO: will eventually add links to an appropriate shapefile analysis program in the scripts. Still in development.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        chkGenShapeStats->setText(QString());
        label_4->setText(QApplication::translate("frmMainClass", "Register in Database?", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        chkDatabaseRegister->setStatusTip(QApplication::translate("frmMainClass", "TODO: register runs in database for consolidation.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        chkDatabaseRegister->setText(QString());
        label_5->setText(QApplication::translate("frmMainClass", "Summary:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        txtSummary->setStatusTip(QApplication::translate("frmMainClass", "The summary of the options", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        label_24->setText(QApplication::translate("frmMainClass", "Compress Results?", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        chkCompress->setStatusTip(QApplication::translate("frmMainClass", "Once script runs, compress results? Generally a good idea.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        chkCompress->setText(QString());
        label_25->setText(QApplication::translate("frmMainClass", "Export:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        chkDoExport->setStatusTip(QApplication::translate("frmMainClass", "Once each ddscat invocation finishes, copy output to this directory. Useful for consolidation.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        chkDoExport->setText(QString());
#ifndef QT_NO_STATUSTIP
        txtExportDir->setStatusTip(QApplication::translate("frmMainClass", "Once each ddscat invocation finishes, copy output to this directory. Useful for consolidation.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        txtExportDir->setPlaceholderText(QApplication::translate("frmMainClass", "/data/rhoneyag/incoming", 0, QApplication::UnicodeUTF8));
        cmdSelectExportDir->setText(QApplication::translate("frmMainClass", "...", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tabGeneral), QApplication::translate("frmMainClass", "General", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("frmMainClass", "Torque Calculations?", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        chkDoTorques->setStatusTip(QApplication::translate("frmMainClass", "Either do or skip torque calculations", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        chkDoTorques->setText(QString());
        label_12->setText(QApplication::translate("frmMainClass", "Solution Method:", 0, QApplication::UnicodeUTF8));
        cmbSolnMeth->clear();
        cmbSolnMeth->insertItems(0, QStringList()
         << QApplication::translate("frmMainClass", "PBCGS2", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmMainClass", "PBCGST", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmMainClass", "PETRKP", 0, QApplication::UnicodeUTF8)
        );
#ifndef QT_NO_STATUSTIP
        cmbSolnMeth->setStatusTip(QApplication::translate("frmMainClass", "CMDSOL*6 (PBCGS2, PBCGST, PETRKP) -- select solution method", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        label_13->setText(QApplication::translate("frmMainClass", "FFT Solver Method:", 0, QApplication::UnicodeUTF8));
        cmbFFTsolver->clear();
        cmbFFTsolver->insertItems(0, QStringList()
         << QApplication::translate("frmMainClass", "GPFAFT", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmMainClass", "FFTMKL", 0, QApplication::UnicodeUTF8)
        );
#ifndef QT_NO_STATUSTIP
        cmbFFTsolver->setStatusTip(QApplication::translate("frmMainClass", "CMDFFT*6 (GPFAFT, FFTMKL). FFTMKL only available on Intel-compiled DDSCAT 7.2.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        label_14->setText(QApplication::translate("frmMainClass", "Polarizability Setup (CALPHA):", 0, QApplication::UnicodeUTF8));
        cmbCALPHA->clear();
        cmbCALPHA->insertItems(0, QStringList()
         << QApplication::translate("frmMainClass", "GKDLDR", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmMainClass", "LATTDIR", 0, QApplication::UnicodeUTF8)
        );
#ifndef QT_NO_STATUSTIP
        cmbCALPHA->setStatusTip(QApplication::translate("frmMainClass", "CALPHA*6 (GKDLDR, LATTDR)", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        label_15->setText(QApplication::translate("frmMainClass", "Binning:", 0, QApplication::UnicodeUTF8));
        cmbBinning->clear();
        cmbBinning->insertItems(0, QStringList()
         << QApplication::translate("frmMainClass", "NOTBIN", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmMainClass", "ORIBIN", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmMainClass", "ALLBIN", 0, QApplication::UnicodeUTF8)
        );
#ifndef QT_NO_STATUSTIP
        cmbBinning->setStatusTip(QApplication::translate("frmMainClass", "CBINFLAG (NOTBIN, ORIBIN, ALLBIN)", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        label_16->setText(QApplication::translate("frmMainClass", "Initial Memory Dimensioning:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        txtImem1->setStatusTip(QApplication::translate("frmMainClass", "Memory cells in each direction reserved for the first iteration. After the first, the matrix is autosized.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        txtImem1->setPlaceholderText(QApplication::translate("frmMainClass", "101", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        txtImem2->setStatusTip(QApplication::translate("frmMainClass", "Memory cells in each direction reserved for the first iteration. After the first, the matrix is autosized.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        txtImem2->setPlaceholderText(QApplication::translate("frmMainClass", "101", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        txtImem3->setStatusTip(QApplication::translate("frmMainClass", "Memory cells in each direction reserved for the first iteration. After the first, the matrix is autosized.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        txtImem3->setPlaceholderText(QApplication::translate("frmMainClass", "101", 0, QApplication::UnicodeUTF8));
        label_17->setText(QApplication::translate("frmMainClass", "Do Additional Nearfield Calcs?", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        chkNearfield->setStatusTip(QApplication::translate("frmMainClass", "Calculate nearfield E?", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        chkNearfield->setText(QString());
        label_18->setText(QApplication::translate("frmMainClass", "Nearfield fract stuff:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        txtNear1->setStatusTip(QApplication::translate("frmMainClass", "fract. extens. of calc. vol. in -x", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        txtNear1->setPlaceholderText(QApplication::translate("frmMainClass", "0.0", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        txtNear2->setStatusTip(QApplication::translate("frmMainClass", "fract. extens. of calc. vol. in +x", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        txtNear2->setPlaceholderText(QApplication::translate("frmMainClass", "0.0", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        txtNear3->setStatusTip(QApplication::translate("frmMainClass", "fract. extens. of calc. vol. in -y", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        txtNear3->setPlaceholderText(QApplication::translate("frmMainClass", "0.0", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        txtNear4->setStatusTip(QApplication::translate("frmMainClass", "fract. extens. of calc. vol. in +y", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        txtNear4->setPlaceholderText(QApplication::translate("frmMainClass", "0.0", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        txtNear5->setStatusTip(QApplication::translate("frmMainClass", "fract. extens. of calc. vol. in -z", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        txtNear5->setPlaceholderText(QApplication::translate("frmMainClass", "0.0", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        txtNear6->setStatusTip(QApplication::translate("frmMainClass", "fract. extens. of calc. vol. in +z", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        txtNear6->setPlaceholderText(QApplication::translate("frmMainClass", "0.0", 0, QApplication::UnicodeUTF8));
        label_19->setText(QApplication::translate("frmMainClass", "Max Error Tolerance:", 0, QApplication::UnicodeUTF8));
        label_20->setText(QApplication::translate("frmMainClass", "Max Number of Iterations:", 0, QApplication::UnicodeUTF8));
        label_22->setText(QApplication::translate("frmMainClass", "Interaction Cutoff Parameter (GAMMA):", 0, QApplication::UnicodeUTF8));
        label_21->setText(QApplication::translate("frmMainClass", "Angular Resolution (ETASCA):", 0, QApplication::UnicodeUTF8));
        label_23->setText(QApplication::translate("frmMainClass", "Refractive Index of Ambient Medium:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        txtMaxTol->setStatusTip(QApplication::translate("frmMainClass", "TOL = MAX ALLOWED (NORM OF |G>=AC|E>-ACA|X>)/(NORM OF AC|E>)", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        txtMaxTol->setPlaceholderText(QApplication::translate("frmMainClass", "1.0E-5", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        txtMaxIter->setStatusTip(QApplication::translate("frmMainClass", "Maximum number of iterations allowed", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        txtMaxIter->setPlaceholderText(QApplication::translate("frmMainClass", "300", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        txtGamma->setStatusTip(QApplication::translate("frmMainClass", "Interaction cutoff parameter for PBC calculations (1e-2 is normal, 3e-3 for greater accuracy)", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        txtGamma->setPlaceholderText(QApplication::translate("frmMainClass", "5.00e-3", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        txtETASCA->setStatusTip(QApplication::translate("frmMainClass", "Angular resolution for calculation of <cos>, etc... (number of angles is proportional to [(3+x)/ETASCA]^2 )", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        txtETASCA->setPlaceholderText(QApplication::translate("frmMainClass", "0.5", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        txtNAMBIENT->setStatusTip(QApplication::translate("frmMainClass", "Refractive Index of Ambient Medium", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        txtNAMBIENT->setPlaceholderText(QApplication::translate("frmMainClass", "1.000", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("frmMainClass", "Basics", 0, QApplication::UnicodeUTF8));
        label_26->setText(QApplication::translate("frmMainClass", "Target Type:", 0, QApplication::UnicodeUTF8));
        label_27->setText(QApplication::translate("frmMainClass", "Shape File Source:", 0, QApplication::UnicodeUTF8));
        label_28->setText(QApplication::translate("frmMainClass", "Shape Parameters:", 0, QApplication::UnicodeUTF8));
        label_31->setText(QApplication::translate("frmMainClass", "Dielectric Materials:", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem = tableWidget_3->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QApplication::translate("frmMainClass", "Substance", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem1 = tableWidget_3->verticalHeaderItem(0);
        ___qtablewidgetitem1->setText(QApplication::translate("frmMainClass", "1", 0, QApplication::UnicodeUTF8));

        const bool __sortingEnabled = tableWidget_3->isSortingEnabled();
        tableWidget_3->setSortingEnabled(false);
        QTableWidgetItem *___qtablewidgetitem2 = tableWidget_3->item(0, 0);
        ___qtablewidgetitem2->setText(QApplication::translate("frmMainClass", "Ice", 0, QApplication::UnicodeUTF8));
        tableWidget_3->setSortingEnabled(__sortingEnabled);

        label_29->setText(QApplication::translate("frmMainClass", "Interdipole Spacing:", 0, QApplication::UnicodeUTF8));
        lineEdit_7->setPlaceholderText(QApplication::translate("frmMainClass", "263", 0, QApplication::UnicodeUTF8));
        comboBox_2->clear();
        comboBox_2->insertItems(0, QStringList()
         << QApplication::translate("frmMainClass", "K", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmMainClass", "C", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("frmMainClass", "F", 0, QApplication::UnicodeUTF8)
        );
        label_30->setText(QApplication::translate("frmMainClass", "Temperature:", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("frmMainClass", "Target", 0, QApplication::UnicodeUTF8));
        label_32->setText(QApplication::translate("frmMainClass", "Frequencies:", 0, QApplication::UnicodeUTF8));
        pushButton_5->setText(QApplication::translate("frmMainClass", "PushButton", 0, QApplication::UnicodeUTF8));
        pushButton_2->setText(QApplication::translate("frmMainClass", "PushButton", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("frmMainClass", "PushButton", 0, QApplication::UnicodeUTF8));
        label_33->setText(QApplication::translate("frmMainClass", "Target Sizes:", 0, QApplication::UnicodeUTF8));
        pushButton_4->setText(QApplication::translate("frmMainClass", "PushButton", 0, QApplication::UnicodeUTF8));
        pushButton_3->setText(QApplication::translate("frmMainClass", "PushButton", 0, QApplication::UnicodeUTF8));
        pushButton_6->setText(QApplication::translate("frmMainClass", "PushButton", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("frmMainClass", "Size and Frequency", 0, QApplication::UnicodeUTF8));
        label_34->setText(QApplication::translate("frmMainClass", "Rotations:", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem3 = tableWidget->horizontalHeaderItem(0);
        ___qtablewidgetitem3->setText(QApplication::translate("frmMainClass", "Min", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem4 = tableWidget->horizontalHeaderItem(1);
        ___qtablewidgetitem4->setText(QApplication::translate("frmMainClass", "Max", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem5 = tableWidget->horizontalHeaderItem(2);
        ___qtablewidgetitem5->setText(QApplication::translate("frmMainClass", "Number", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem6 = tableWidget->verticalHeaderItem(0);
        ___qtablewidgetitem6->setText(QApplication::translate("frmMainClass", "Beta", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem7 = tableWidget->verticalHeaderItem(1);
        ___qtablewidgetitem7->setText(QApplication::translate("frmMainClass", "Theta", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem8 = tableWidget->verticalHeaderItem(2);
        ___qtablewidgetitem8->setText(QApplication::translate("frmMainClass", "Phi", 0, QApplication::UnicodeUTF8));
        label_36->setText(QApplication::translate("frmMainClass", "Rotation Splitting:", 0, QApplication::UnicodeUTF8));
        checkBox->setText(QString());
        label_35->setText(QApplication::translate("frmMainClass", "Scattering Angles:", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem9 = tableWidget_2->horizontalHeaderItem(0);
        ___qtablewidgetitem9->setText(QApplication::translate("frmMainClass", "Phi", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem10 = tableWidget_2->horizontalHeaderItem(1);
        ___qtablewidgetitem10->setText(QApplication::translate("frmMainClass", "Theta_min", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem11 = tableWidget_2->horizontalHeaderItem(2);
        ___qtablewidgetitem11->setText(QApplication::translate("frmMainClass", "Theta_max", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem12 = tableWidget_2->horizontalHeaderItem(3);
        ___qtablewidgetitem12->setText(QApplication::translate("frmMainClass", "d_Theta", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem13 = tableWidget_2->verticalHeaderItem(0);
        ___qtablewidgetitem13->setText(QApplication::translate("frmMainClass", "Plane 1", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem14 = tableWidget_2->verticalHeaderItem(1);
        ___qtablewidgetitem14->setText(QApplication::translate("frmMainClass", "Plane 2", 0, QApplication::UnicodeUTF8));

        const bool __sortingEnabled1 = tableWidget_2->isSortingEnabled();
        tableWidget_2->setSortingEnabled(false);
        tableWidget_2->setSortingEnabled(__sortingEnabled1);

        tabWidget->setTabText(tabWidget->indexOf(tab_4), QApplication::translate("frmMainClass", "Rotations and Scattering Angles", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        cmdGenerate->setStatusTip(QApplication::translate("frmMainClass", "Click to generate the files and directory structure", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        cmdGenerate->setText(QApplication::translate("frmMainClass", "Generate Files", 0, QApplication::UnicodeUTF8));
        menu_File->setTitle(QApplication::translate("frmMainClass", "&File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class frmMainClass: public Ui_frmMainClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FRMMAIN_H
