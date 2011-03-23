/********************************************************************************
** Form generated from reading UI file 'frmabout.ui'
**
** Created: Mon Mar 21 14:22:39 2011
**      by: Qt User Interface Compiler version 4.7.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FRMABOUT_H
#define UI_FRMABOUT_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QFormLayout>
#include <QtGui/QGraphicsView>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QTextBrowser>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_frmAbout
{
public:
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QFormLayout *formLayout_2;
    QGraphicsView *graphicsView;
    QLabel *label;
    QLabel *label_2;
    QTextBrowser *txtLibProps;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *cmdClose;

    void setupUi(QDialog *frmAbout)
    {
        if (frmAbout->objectName().isEmpty())
            frmAbout->setObjectName(QString::fromUtf8("frmAbout"));
        frmAbout->resize(400, 300);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(frmAbout->sizePolicy().hasHeightForWidth());
        frmAbout->setSizePolicy(sizePolicy);
        frmAbout->setMinimumSize(QSize(400, 300));
        frmAbout->setMaximumSize(QSize(400, 300));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/rtlogo"), QSize(), QIcon::Normal, QIcon::Off);
        frmAbout->setWindowIcon(icon);
        verticalLayoutWidget = new QWidget(frmAbout);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(10, 10, 381, 281));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        formLayout_2 = new QFormLayout();
        formLayout_2->setObjectName(QString::fromUtf8("formLayout_2"));
        graphicsView = new QGraphicsView(verticalLayoutWidget);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));
        graphicsView->setMaximumSize(QSize(60, 60));

        formLayout_2->setWidget(0, QFormLayout::LabelRole, graphicsView);

        label = new QLabel(verticalLayoutWidget);
        label->setObjectName(QString::fromUtf8("label"));

        formLayout_2->setWidget(0, QFormLayout::FieldRole, label);

        label_2 = new QLabel(verticalLayoutWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        formLayout_2->setWidget(1, QFormLayout::LabelRole, label_2);

        txtLibProps = new QTextBrowser(verticalLayoutWidget);
        txtLibProps->setObjectName(QString::fromUtf8("txtLibProps"));

        formLayout_2->setWidget(1, QFormLayout::FieldRole, txtLibProps);


        verticalLayout->addLayout(formLayout_2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_2);

        cmdClose = new QPushButton(verticalLayoutWidget);
        cmdClose->setObjectName(QString::fromUtf8("cmdClose"));
        cmdClose->setDefault(true);
        cmdClose->setFlat(false);

        horizontalLayout->addWidget(cmdClose);


        verticalLayout->addLayout(horizontalLayout);


        retranslateUi(frmAbout);
        QObject::connect(cmdClose, SIGNAL(clicked()), frmAbout, SLOT(close()));

        QMetaObject::connectSlotsByName(frmAbout);
    } // setupUi

    void retranslateUi(QDialog *frmAbout)
    {
        frmAbout->setWindowTitle(QApplication::translate("frmAbout", "About rtmath", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("frmAbout", "rtmath Version______", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("frmAbout", "Library Properties:", 0, QApplication::UnicodeUTF8));
        cmdClose->setText(QApplication::translate("frmAbout", "Close", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class frmAbout: public Ui_frmAbout {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FRMABOUT_H
