#ifndef FRMMAIN_H
#define FRMMAIN_H

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
		void menuFreqs(const QPoint &);
		void menuSizes(const QPoint &);
		void menuRots(const QPoint &);
		void menuTemps(const QPoint &);
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

#endif // FRMMAIN_H
