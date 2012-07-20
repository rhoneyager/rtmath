#ifndef FRMMAIN_H
#define FRMMAIN_H
#pragma warning( disable : 4996 )

#include <QMainWindow>
#include "ui_frmmain.h"
#include "../../rtmath/rtmath/ddscat/ddparGenerator.h"

class frmMain : public QMainWindow
{
	Q_OBJECT

public:
	frmMain(QWidget *parent = 0, Qt::WFlags flags = 0);
	~frmMain();

private:
	Ui::frmMainClass ui;
	private slots:
		void allowExport(int);
		void editTreeItem(QTreeWidgetItem*,int);
		void menuGlobals(const QPoint &);
		void menuRots(const QPoint &);
		//void menuTargets(const QPoint &);
		void menuScaAngles(const QPoint &);
		void generateRuns();
		void loadSet();
		void saveSet();
		void newSet();
		void import();
		/*
		void menuTargets(const QPoint &);
		void menuDielectrics(const QPoint &);
		*/
	private:
		void toGenerator();
		void fromGenerator();

		rtmath::ddscat::ddParGenerator _gen;
};


template < class T >
T getValText(QLineEdit *src);

template <> int getValText(QLineEdit *src);
template <> size_t getValText(QLineEdit *src);
template <> double getValText(QLineEdit *src);

#endif // FRMMAIN_H
