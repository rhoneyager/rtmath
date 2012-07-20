#include "stdafx.h"
#include "frmmain.h"
#include "../../rtmath/rtmath/ROOTlink.h"
#include "../../rtmath/rtmath/qt_funcs.h"
#include "../../rtmath/rtmath/serialization.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include <QFileDialog>

template <>
int getValText(QLineEdit *src)
{
	return src->text().size() ? src->text().toInt() : src->placeholderText().toInt() ;
}

template <>
size_t getValText(QLineEdit *src)
{
	return src->text().size() ? src->text().toInt() : src->placeholderText().toInt() ;
}

template <>
double getValText(QLineEdit *src)
{
	return src->text().size() ? src->text().toDouble() : src->placeholderText().toDouble() ;
}

frmMain::frmMain(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	ui.setupUi(this);
	//ui.treeFreqs->setContextMenuPolicy
	for (int i=0; i< ui.treeRots->columnCount(); i++)
		ui.treeRots->resizeColumnToContents(i);
}

frmMain::~frmMain()
{

}

void frmMain::allowExport(int val)
{
	ui.txtExportDir->setEnabled((bool) val);
}

void frmMain::editTreeItem(QTreeWidgetItem* item,int column)
{
	item->setFlags(item->flags() | Qt::ItemIsEditable);
	item->treeWidget()->editItem(item,column);
	//ui.treeFreqs->editItem(item, column);
}

void frmMain::menuGlobals(const QPoint &p)
{
	QPoint pos = ui.treeGlobals->mapToGlobal(p);
	QMenu mnuF;
	QAction *mAdd, *mRemove;
	mAdd = mnuF.addAction("&Add");
	mRemove = mnuF.addAction("&Remove");

	QList<QTreeWidgetItem *> selFreqs = ui.treeGlobals->selectedItems();
	(selFreqs.size())? mRemove->setEnabled(true) : mRemove->setEnabled(false);

	QAction *selected = mnuF.exec(pos);
	if (selected)
	{
		// Will manipulate items and their mapped frequency entries (text strings)
		if (selected == mAdd)
		{
			QTreeWidgetItem *nitem = new QTreeWidgetItem();
			nitem->setText(0, "aeff");
			nitem->setText(1, "100:50:350");
			nitem->setText(2, "um");
			ui.treeGlobals->addTopLevelItem(nitem);
		}
		if (selected == mRemove)
		{
			// Drop the selected items
			for (auto it = selFreqs.begin(); it != selFreqs.end(); it++)
				delete *it;
		}
	}
}

void frmMain::menuRots(const QPoint &p)
{
	QPoint pos = ui.treeRots->mapToGlobal(p);
	QMenu mnuF;
	QAction *mAdd, *mRemove;
	mAdd = mnuF.addAction("&Add");
	mRemove = mnuF.addAction("&Remove");

	QList<QTreeWidgetItem *> selFreqs = ui.treeRots->selectedItems();
	(selFreqs.size())? mRemove->setEnabled(true) : mRemove->setEnabled(false);

	QAction *selected = mnuF.exec(pos);
	if (selected)
	{
		// Will manipulate items and their mapped frequency entries (text strings)
		if (selected == mAdd)
		{
			QTreeWidgetItem *nitem = new QTreeWidgetItem();
			nitem->setText(0, "0");
			nitem->setText(1, "360");
			nitem->setText(2, "6");
			nitem->setText(3, "0");
			nitem->setText(4, "180");
			nitem->setText(5, "10");
			nitem->setText(6, "0");
			nitem->setText(7, "360");
			nitem->setText(8, "6");
			ui.treeRots->addTopLevelItem(nitem);
		}
		if (selected == mRemove)
		{
			// Drop the selected items
			for (auto it = selFreqs.begin(); it != selFreqs.end(); it++)
				delete *it;
		}
	}
}

void frmMain::menuScaAngles(const QPoint &p)
{
	QPoint pos = ui.treeScattAngles->mapToGlobal(p);
	QMenu mnuF;
	QAction *mAdd, *mRemove;
	mAdd = mnuF.addAction("&Add Plane");
	mRemove = mnuF.addAction("&Remove Plane");

	QList<QTreeWidgetItem *> selFreqs = ui.treeScattAngles->selectedItems();
	(selFreqs.size())? mRemove->setEnabled(true) : mRemove->setEnabled(false);

	QAction *selected = mnuF.exec(pos);
	if (selected)
	{
		// Will manipulate items and their mapped frequency entries (text strings)
		if (selected == mAdd)
		{
			QTreeWidgetItem *nitem = new QTreeWidgetItem();
			nitem->setText(0, "0");
			nitem->setText(1, "0");
			nitem->setText(2, "180");
			nitem->setText(3, "10");
			ui.treeScattAngles->addTopLevelItem(nitem);
		}
		if (selected == mRemove)
		{
			// Drop the selected items
			for (auto it = selFreqs.begin(); it != selFreqs.end(); it++)
				delete *it;
		}
	}
}

void frmMain::newSet()
{
	rtmath::ddscat::ddParGenerator ng;
	_gen = ng;
	fromGenerator();
}

void frmMain::loadSet()
{
	// Open dialog box asking from where
	
	QFileDialog dialog(this);
	dialog.setFileMode(QFileDialog::ExistingFile);
	dialog.setNameFilter(tr("XML files (*.xml)"));
	dialog.setViewMode(QFileDialog::Detail);
	
	if (dialog.exec())
	{
		QStringList qfilename;
		qfilename = dialog.selectedFiles();

		// Have generator load
		std::string filename = qfilename.begin()->toStdString();

		rtmath::serialization::read<rtmath::ddscat::ddParGenerator>(_gen,filename);
		// Update ui
		fromGenerator();
	}
}

void frmMain::saveSet()
{
	toGenerator();
	// Now, open a file dialog asking for the save location
	QFileDialog dialog(this);
	dialog.setFileMode(QFileDialog::AnyFile);
	dialog.setNameFilter(tr("XML files (*.xml)"));
	dialog.setViewMode(QFileDialog::Detail);
	// Have the generator save
	if (dialog.exec())
	{
		QStringList qfilename = dialog.selectedFiles();
		std::string filename = qfilename.begin()->toStdString();

		rtmath::serialization::write<rtmath::ddscat::ddParGenerator>(_gen,filename);
	}
}

void frmMain::import()
{
	QFileDialog dialog(this);
	dialog.setFileMode(QFileDialog::ExistingFile);
	dialog.setNameFilter(tr("PAR files (*.par)"));
	dialog.setViewMode(QFileDialog::Detail);
	
	if (dialog.exec())
	{
		QStringList qfilename;
		qfilename = dialog.selectedFiles();

		// Have generator load
		std::string filename = qfilename.begin()->toStdString();

		newSet();
		_gen.import(filename);
		// Update ui
		fromGenerator();
	}
}

void frmMain::generateRuns()
{
	// Translate all of the information into stl objects and then invoke the stl functions
	// Note: the library call should be in a separate thread, to allow for better 
	// responsiveness as files are being created
	toGenerator();
	std::string dname;
	ui.txtOutLocation->text().size() ? dname = ui.txtOutLocation->text().toStdString() : dname = ui.txtOutLocation->placeholderText().toStdString();
	_gen.generate(dname);
	rtmath::serialization::write<rtmath::ddscat::ddParGenerator>(_gen,dname);
}

void frmMain::toGenerator()
{
	_gen.name = ui.txtRunName->text().toStdString();
	_gen.description = ui.txtDescription->toPlainText().toStdString();
	_gen.outLocation = ui.txtOutLocation->text().toStdString();
	//_gen.ddscatVer;
	//_gen.strPreCmds; // TODO
	//_gen.strPostCdms; // TODO
	_gen.baseParFile = ui.txtBaseFile->text().toStdString();
	
	// shape stuff
	//_gen._shapeBase; // TODO
	// reg stuff
	_gen.compressResults = ui.chkCompress->isChecked();
	_gen.genIndivScripts = ui.chkGenIndivRunScripts->isChecked();
	_gen.genMassScript = ui.chkGenMassRunScript->isChecked();
	_gen.shapeStats = ui.chkGenShapeStats->isChecked();
	_gen.registerDatabase = ui.chkDatabaseRegister->isChecked();
	_gen.doExport = ui.chkDoExport->isChecked();
	_gen.exportLoc = ui.txtExportDir->text().toStdString();
	
	_gen.base.doTorques(ui.chkDoTorques->isChecked());
	_gen.base.setSolnMeth(ui.cmbSolnMeth->currentText().toStdString());
	_gen.base.setFFTsolver(ui.cmbFFTsolver->currentText().toStdString());
	_gen.base.setCalpha(ui.cmbCALPHA->currentText().toStdString());
	_gen.base.setBinning(ui.cmbBinning->currentText().toStdString());

	_gen.base.Imem(0, getValText<int>(this->ui.txtImem1) );
	_gen.base.Imem(1, getValText<int>(this->ui.txtImem2) );
	_gen.base.Imem(2, getValText<int>(this->ui.txtImem3) );

	_gen.base.doNearField(ui.chkNearfield->isChecked());

	_gen.base.near(0, getValText<double>(ui.txtNear1) );
	_gen.base.near(1, getValText<double>(ui.txtNear2) );
	_gen.base.near(2, getValText<double>(ui.txtNear3) );
	_gen.base.near(3, getValText<double>(ui.txtNear4) );
	_gen.base.near(4, getValText<double>(ui.txtNear5) );
	_gen.base.near(5, getValText<double>(ui.txtNear6) );

	_gen.base.maxTol(getValText<double>(ui.txtMaxTol) );
	_gen.base.maxIter(getValText<size_t>(ui.txtMaxIter) );
	_gen.base.gamma(getValText<double>(ui.txtGamma) );
	_gen.base.etasca(getValText<double>(ui.txtETASCA) );
	_gen.base.nAmbient(getValText<double>(ui.txtNAMBIENT) );
	_gen.base.writeSca( ui.chkWriteSca->isChecked());

	// Global properties
	{
		_gen.shapeConstraints.clear();

		QTreeWidgetItem *i = ui.treeGlobals->takeTopLevelItem(0);
		while (i)
		{
			// Can get info from each column as a text string.
			// Col 0 is var name
			// Col 1 is range
			// Col 2 is units
			std::string vname, range, units;
			vname = i->text(0).toStdString();
			range = i->text(1).toStdString();
			units = i->text(2).toStdString();

			using namespace rtmath::ddscat;
			_gen.addConstraint(shapeConstraint::create(vname, range, units));

			i = ui.treeGlobals->itemBelow(i);
		}
	}

	GETOBJKEY();
	// Target properties
	// Need to use shape matching / casting function
	{
	}

	// Rotations
	{
		_gen.rots.clear();
		QTreeWidgetItem *i = ui.treeScattAngles->takeTopLevelItem(0);
		while (i)
		{
			double bMin, bMax, tMin, tMax, pMin, pMax;
			size_t bN, tN, pN;

			bMin = i->text(0).toDouble();
			bMax = i->text(1).toDouble();
			bN = i->text(2).toInt();
			tMin = i->text(3).toDouble();
			tMax = i->text(4).toDouble();
			tN = i->text(5).toInt();
			pMin = i->text(6).toDouble();
			pMax = i->text(7).toDouble();
			pN = i->text(8).toInt();

			_gen.rots.insert( rtmath::ddscat::rotations::create(bMin,bMax,bN,tMin,tMax,tN,pMin,pMax,pN) );

			i = ui.treeScattAngles->itemBelow(i);
		}
	}

	// Scattering angles
	{
		size_t n = 0;

		QTreeWidgetItem *i = ui.treeScattAngles->takeTopLevelItem(0);
		while (i)
		{
			n++;
			// Can get info from each column as a text string.
			// phi, theta_min, theta_max, ntheta
			double phi, thetan_min, thetan_max, dtheta;
			phi = i->text(0).toDouble();
			thetan_min = i->text(1).toDouble();
			thetan_max = i->text(2).toDouble();
			dtheta = i->text(3).toDouble();

			using namespace rtmath::ddscat;
			_gen.base.setPlane(n, phi, thetan_min, thetan_max, dtheta);

			i = ui.treeScattAngles->itemBelow(i);
		}
		// Set number of scattering planes after iteration
		_gen.base.numPlanes(n);
	}
	
}

void frmMain::fromGenerator()
{
	GETOBJKEY();
}

