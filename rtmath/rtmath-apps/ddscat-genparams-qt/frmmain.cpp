#include "stdafx.h"
#include "frmmain.h"
#include "../../rtmath/rtmath/ROOTlink.h"
#include "../../rtmath/rtmath/qt_funcs.h"
#include <QFileDialog>

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
	
	ui.treeFreqs->editItem(item, column);
}

void frmMain::menuFreqs(const QPoint &p)
{
	QPoint pos = ui.treeFreqs->mapToGlobal(p);
	QMenu mnuF;
	QAction *mAdd, *mRemove;
	mAdd = mnuF.addAction("&Add");
	mRemove = mnuF.addAction("&Remove");

	QList<QTreeWidgetItem *> selFreqs = ui.treeFreqs->selectedItems();
	(selFreqs.size())? mRemove->setEnabled(true) : mRemove->setEnabled(false);

	QAction *selected = mnuF.exec(pos);
	if (selected)
	{
		// Will manipulate items and their mapped frequency entries (text strings)
		if (selected == mAdd)
		{
			QTreeWidgetItem *nitem = new QTreeWidgetItem();
			nitem->setText(0, "New range");
			nitem->setText(1, "GHz");
			ui.treeFreqs->addTopLevelItem(nitem);
		}
		if (selected == mRemove)
		{
			// Drop the selected items
			for (auto it = selFreqs.begin(); it != selFreqs.end(); it++)
				delete *it;
		}
	}
}

void frmMain::menuSizes(const QPoint &p)
{
	QPoint pos = ui.treeSizes->mapToGlobal(p);
	QMenu mnuF;
	QAction *mAdd, *mRemove;
	mAdd = mnuF.addAction("&Add");
	mRemove = mnuF.addAction("&Remove");

	QList<QTreeWidgetItem *> selFreqs = ui.treeSizes->selectedItems();
	(selFreqs.size())? mRemove->setEnabled(true) : mRemove->setEnabled(false);

	QAction *selected = mnuF.exec(pos);
	if (selected)
	{
		// Will manipulate items and their mapped frequency entries (text strings)
		if (selected == mAdd)
		{
			QTreeWidgetItem *nitem = new QTreeWidgetItem();
			nitem->setText(0, "Effective Radius");
			nitem->setText(1, "New range");
			nitem->setText(2, "um");
			ui.treeSizes->addTopLevelItem(nitem);
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

void frmMain::menuTemps(const QPoint &p)
{
	QPoint pos = ui.treeTemps->mapToGlobal(p);
	QMenu mnuF;
	QAction *mAdd, *mRemove;
	mAdd = mnuF.addAction("&Add");
	mRemove = mnuF.addAction("&Remove");

	QList<QTreeWidgetItem *> selFreqs = ui.treeTemps->selectedItems();
	(selFreqs.size())? mRemove->setEnabled(true) : mRemove->setEnabled(false);

	QAction *selected = mnuF.exec(pos);
	if (selected)
	{
		// Will manipulate items and their mapped frequency entries (text strings)
		if (selected == mAdd)
		{
			QTreeWidgetItem *nitem = new QTreeWidgetItem();
			nitem->setText(0, "263");
			nitem->setText(1, "K");
			ui.treeTemps->addTopLevelItem(nitem);
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

		rtmath::ddscat::ddParGenerator::read(_gen,filename);
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

		rtmath::ddscat::ddParGenerator::write(_gen,filename);
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
        rtmath::ddscat::ddParGenerator::write(_gen,dname);
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
	
	ui.txtImem1->text().size() ? _gen.Imem1 = ui.txtImem1->text().toInt() : _gen.Imem1 = ui.txtImem1->placeholderText().toInt();
	ui.txtImem2->text().size() ? _gen.Imem2 = ui.txtImem2->text().toInt() : _gen.Imem1 = ui.txtImem2->placeholderText().toInt();
	ui.txtImem3->text().size() ? _gen.Imem3 = ui.txtImem3->text().toInt() : _gen.Imem1 = ui.txtImem3->placeholderText().toInt();
	_gen.doNearField = ui.chkNearfield->isChecked();
	ui.txtNear1->text().size() ? _gen.near1 = ui.txtNear1->text().toDouble() : _gen.near1 = ui.txtNear1->placeholderText().toDouble();
	ui.txtNear2->text().size() ? _gen.near2 = ui.txtNear2->text().toDouble() : _gen.near2 = ui.txtNear2->placeholderText().toDouble();
	ui.txtNear3->text().size() ? _gen.near3 = ui.txtNear3->text().toDouble() : _gen.near3 = ui.txtNear3->placeholderText().toDouble();
	ui.txtNear4->text().size() ? _gen.near4 = ui.txtNear4->text().toDouble() : _gen.near4 = ui.txtNear4->placeholderText().toDouble();
	ui.txtNear5->text().size() ? _gen.near5 = ui.txtNear5->text().toDouble() : _gen.near5 = ui.txtNear5->placeholderText().toDouble();
	ui.txtNear6->text().size() ? _gen.near6 = ui.txtNear6->text().toDouble() : _gen.near6 = ui.txtNear6->placeholderText().toDouble();
	ui.txtMaxTol->text().size() ? _gen.maxTol = ui.txtMaxTol->text().toDouble() : _gen.maxTol = ui.txtMaxTol->placeholderText().toDouble();
	ui.txtMaxIter->text().size() ? _gen.maxIter = ui.txtMaxIter->text().toDouble() : _gen.maxIter =  ui.txtMaxIter->placeholderText().toDouble();
	ui.txtGamma->text().size() ? _gen.gamma = ui.txtGamma->text().toDouble() : _gen.gamma =  ui.txtGamma->placeholderText().toDouble();
	ui.txtETASCA->text().size() ? _gen.etasca = ui.txtETASCA->text().toDouble() : _gen.etasca =  ui.txtETASCA->placeholderText().toDouble();
	ui.txtNAMBIENT->text().size() ? _gen.nambient = ui.txtNAMBIENT->text().toDouble() : _gen.nambient =  ui.txtNAMBIENT->placeholderText().toDouble();

	_gen.doTorques = ui.chkDoTorques->isChecked();
	_gen.solnMeth = ui.cmbSolnMeth->currentText().toStdString();
	_gen.FFTsolver = ui.cmbFFTsolver->currentText().toStdString();
	_gen.Calpha = ui.cmbCALPHA->currentText().toStdString();
	_gen.binning = ui.cmbBinning->currentText().toStdString();

	MARK();
	// Target types

	// Temps

	// Freqs

	// Sizes

	// Rotations

	// Scattering angles
}

void frmMain::fromGenerator()
{
	MARK();
}

