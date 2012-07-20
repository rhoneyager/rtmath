/********************************************************************************
** Form generated from reading UI file 'frmTarget.ui'
**
** Created: Fri Jul 20 04:21:43 2012
**      by: Qt User Interface Compiler version 4.8.1
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
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QTextBrowser>
#include <QtGui/QTreeWidget>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_frmTarget
{
public:
    QWidget *widget;
    QFormLayout *formLayout;
    QLabel *label;
    QComboBox *cmbTargetType;
    QLabel *label_2;
    QLabel *label_3;
    QComboBox *cmbAspectRatio;
    QTextBrowser *textBrowser;
    QTreeWidget *treeConstraints;
    QLabel *label_4;
    QTreeWidget *treeShapedat;
    QWidget *widget1;
    QHBoxLayout *horizontalLayout_2;
    QSpacerItem *horizontalSpacer;
    QPushButton *cmdOK;
    QPushButton *cmdCancel;

    void setupUi(QWidget *frmTarget)
    {
        if (frmTarget->objectName().isEmpty())
            frmTarget->setObjectName(QString::fromUtf8("frmTarget"));
        frmTarget->resize(792, 495);
        widget = new QWidget(frmTarget);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setGeometry(QRect(9, 9, 774, 446));
        formLayout = new QFormLayout(widget);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        formLayout->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(widget);
        label->setObjectName(QString::fromUtf8("label"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label);

        cmbTargetType = new QComboBox(widget);
        cmbTargetType->setObjectName(QString::fromUtf8("cmbTargetType"));

        formLayout->setWidget(1, QFormLayout::FieldRole, cmbTargetType);

        label_2 = new QLabel(widget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        formLayout->setWidget(2, QFormLayout::LabelRole, label_2);

        label_3 = new QLabel(widget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        formLayout->setWidget(3, QFormLayout::LabelRole, label_3);

        cmbAspectRatio = new QComboBox(widget);
        cmbAspectRatio->setObjectName(QString::fromUtf8("cmbAspectRatio"));
        cmbAspectRatio->setEnabled(false);

        formLayout->setWidget(3, QFormLayout::FieldRole, cmbAspectRatio);

        textBrowser = new QTextBrowser(widget);
        textBrowser->setObjectName(QString::fromUtf8("textBrowser"));
        textBrowser->setEnabled(false);

        formLayout->setWidget(4, QFormLayout::FieldRole, textBrowser);

        treeConstraints = new QTreeWidget(widget);
        treeConstraints->setObjectName(QString::fromUtf8("treeConstraints"));
        treeConstraints->setContextMenuPolicy(Qt::CustomContextMenu);
        treeConstraints->setAlternatingRowColors(true);
        treeConstraints->setSelectionMode(QAbstractItemView::ExtendedSelection);

        formLayout->setWidget(0, QFormLayout::FieldRole, treeConstraints);

        label_4 = new QLabel(widget);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label_4);

        treeShapedat = new QTreeWidget(widget);
        treeShapedat->setObjectName(QString::fromUtf8("treeShapedat"));
        treeShapedat->setContextMenuPolicy(Qt::CustomContextMenu);
        treeShapedat->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        treeShapedat->setSelectionMode(QAbstractItemView::ExtendedSelection);
        treeShapedat->header()->setVisible(false);

        formLayout->setWidget(2, QFormLayout::FieldRole, treeShapedat);

        widget1 = new QWidget(frmTarget);
        widget1->setObjectName(QString::fromUtf8("widget1"));
        widget1->setGeometry(QRect(9, 461, 204, 25));
        horizontalLayout_2 = new QHBoxLayout(widget1);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setSizeConstraint(QLayout::SetMinAndMaxSize);
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);

        cmdOK = new QPushButton(widget1);
        cmdOK->setObjectName(QString::fromUtf8("cmdOK"));
        cmdOK->setAutoDefault(true);
        cmdOK->setDefault(true);

        horizontalLayout_2->addWidget(cmdOK);

        cmdCancel = new QPushButton(widget1);
        cmdCancel->setObjectName(QString::fromUtf8("cmdCancel"));

        horizontalLayout_2->addWidget(cmdCancel);


        retranslateUi(frmTarget);
        QObject::connect(treeConstraints, SIGNAL(itemDoubleClicked(QTreeWidgetItem*,int)), frmTarget, SLOT(editTreeItem(QTreeWidgetItem*,int)));
        QObject::connect(treeConstraints, SIGNAL(customContextMenuRequested(QPoint)), frmTarget, SLOT(menuTargetProps(QPoint)));
        QObject::connect(cmdCancel, SIGNAL(clicked()), frmTarget, SLOT(close()));
        QObject::connect(treeShapedat, SIGNAL(itemDoubleClicked(QTreeWidgetItem*,int)), frmTarget, SLOT(editTreeItem(QTreeWidgetItem*,int)));
        QObject::connect(treeShapedat, SIGNAL(customContextMenuRequested(QPoint)), frmTarget, SLOT(menuShapeDatProps(QPoint)));
        QObject::connect(cmbTargetType, SIGNAL(currentIndexChanged(int)), frmTarget, SLOT(targetTypeChanged()));
        QObject::connect(cmdOK, SIGNAL(clicked()), frmTarget, SLOT(processOK()));

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
        QTreeWidgetItem *___qtreewidgetitem = treeConstraints->headerItem();
        ___qtreewidgetitem->setText(2, QApplication::translate("frmTarget", "Units", 0, QApplication::UnicodeUTF8));
        ___qtreewidgetitem->setText(1, QApplication::translate("frmTarget", "Range", 0, QApplication::UnicodeUTF8));
        ___qtreewidgetitem->setText(0, QApplication::translate("frmTarget", "Variable", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        treeConstraints->setToolTip(QString());
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        treeConstraints->setStatusTip(QApplication::translate("frmTarget", "Specify target dimensions. Range notation (min:lin span:max) accepted. Units required.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        label_4->setText(QApplication::translate("frmTarget", "General Target Properties:", 0, QApplication::UnicodeUTF8));
        QTreeWidgetItem *___qtreewidgetitem1 = treeShapedat->headerItem();
        ___qtreewidgetitem1->setText(0, QApplication::translate("frmTarget", "File", 0, QApplication::UnicodeUTF8));
        cmdOK->setText(QApplication::translate("frmTarget", "OK", 0, QApplication::UnicodeUTF8));
        cmdCancel->setText(QApplication::translate("frmTarget", "Cancel", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class frmTarget: public Ui_frmTarget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FRMTARGET_H
