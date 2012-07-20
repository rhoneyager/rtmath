#ifndef FRMMAIN_H
#define FRMMAIN_H
#pragma warning( disable : 4996 )

#include <QMainWindow>
#include <map>
#include "ui_frmmain.h"
#include "frmtarget.h"
#include "../../rtmath/rtmath/ddscat/ddparGenerator.h"
#include "../../rtmath/rtmath/ddscat/shapes.h"

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
		void editTarget(QTreeWidgetItem*,int);
		void menuGlobals(const QPoint &);
		void menuRots(const QPoint &);
		void menuScaAngles(const QPoint &);
		void menuTargets(const QPoint &);
		void generateRuns();
		void loadSet();
		void saveSet();
		void newSet();
		void import();
		void ddverChanged();
		
		/*
		void menuDielectrics(const QPoint &);
		*/
	private:
		void toGenerator();
		void fromGenerator();

		rtmath::ddscat::ddParGenerator _gen;
		int _targetCounter;
		std::map<QTreeWidgetItem*, boost::shared_ptr< rtmath::ddscat::shapeModifiable > > _targets;
		frmTarget _ft;
};


template < class T >
T getValText(QLineEdit *src);

template <> int getValText(QLineEdit *src);
template <> size_t getValText(QLineEdit *src);
template <> double getValText(QLineEdit *src);
template <> std::string getValText(QLineEdit *src);

#endif // FRMMAIN_H
