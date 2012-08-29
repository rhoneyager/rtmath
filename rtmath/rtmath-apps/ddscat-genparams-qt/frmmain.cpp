#include "stdafx.h"
#include "frmmain.h"
#include "frmtarget.h"

#include "../../rtmath/rtmath/ROOTlink.h"
#include "../../rtmath/rtmath/qt_funcs.h"
#include "../../rtmath/rtmath/serialization.h"
#include "../../rtmath/rtmath/Serialization/ddparGenerator_serialization.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"

#include <QFileDialog>
#include <boost/filesystem.hpp>

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

template <>
std::string getValText(QLineEdit *src)
{
	return src->text().size() ? src->text().toStdString() : src->placeholderText().toStdString();
}

frmMain::frmMain(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	ui.setupUi(this);
	//ui.treeFreqs->setContextMenuPolicy
	for (int i=0; i< ui.treeRots->columnCount(); i++)
		ui.treeRots->resizeColumnToContents(i);
	ui.treeTypes->setColumnHidden(1,true);
	_targetCounter = 0;
	fromGenerator();
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

void frmMain::editTarget(QTreeWidgetItem* item,int column)
{
	_ft.setTarget(_targets.at(item));
	_ft.show();
}

void frmMain::menuTargets(const QPoint &p)
{
	QPoint pos = ui.treeTypes->mapToGlobal(p);
	QMenu mnuF;
	QAction *mAdd, *mRemove;
	mAdd = mnuF.addAction("&Add");
	mRemove = mnuF.addAction("&Remove");

	QList<QTreeWidgetItem *> selFreqs = ui.treeTypes->selectedItems();
	(selFreqs.size())? mRemove->setEnabled(true) : mRemove->setEnabled(false);

	QAction *selected = mnuF.exec(pos);
	if (selected)
	{
		// Will manipulate items and their mapped frequency entries (text strings)
		if (selected == mAdd)
		{
			QTreeWidgetItem *nitem = new QTreeWidgetItem();
			//static int i=0;
			//i++;
			//int id = i;
			_targetCounter++;
			nitem->setText(0, QString::number(_targetCounter));
			// And add the appropriate map
			boost::shared_ptr<rtmath::ddscat::shapeModifiable> nshp(
				new rtmath::ddscat::shapeModifiable);
			_targets.insert(std::pair<QTreeWidgetItem*, boost::shared_ptr<rtmath::ddscat::shapeModifiable> >
				(nitem, nshp));

			ui.treeTypes->addTopLevelItem(nitem);
		}
		if (selected == mRemove)
		{
			// Drop the selected items
			for (auto it = selFreqs.begin(); it != selFreqs.end(); it++)
			{
				_targets.erase(*it);

				delete *it;
			}
		}
	}
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

void frmMain::ddverChanged()
{
	// Enable / disable various controls
	size_t ver;
	size_t index = ui.cmbDdver->currentIndex();
	if (index == 0) ver = 72;
	if (index == 1) ver = 70;

	if (ver == 72)
	{
		ui.chkNearfield->setEnabled(true);
		ui.txtNear1->setEnabled(true);
		ui.txtNear2->setEnabled(true);
		ui.txtNear3->setEnabled(true);
		ui.txtNear4->setEnabled(true);
		ui.txtNear5->setEnabled(true);
		ui.txtNear6->setEnabled(true);

		ui.txtMaxIter->setEnabled(true);
		ui.txtNAMBIENT->setEnabled(true);
	} else if (ver == 70)
	{
		ui.chkNearfield->setEnabled(false);
		ui.txtNear1->setEnabled(false);
		ui.txtNear2->setEnabled(false);
		ui.txtNear3->setEnabled(false);
		ui.txtNear4->setEnabled(false);
		ui.txtNear5->setEnabled(false);
		ui.txtNear6->setEnabled(false);

		ui.txtMaxIter->setEnabled(false);
		ui.txtNAMBIENT->setEnabled(false);
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
	QStringList fltrs;
	fltrs << "XML Files (*.xml *.xml.bz2 *.xml.gz)"
		<< "All Files (*.*)";

	dialog.setNameFilters(fltrs);
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
	QStringList fltrs;
	fltrs << "XML Files (*.xml *.xml.bz2 *.xml.gz)"
		<< "All Files (*.*)";

	dialog.setNameFilters(fltrs);
	dialog.setViewMode(QFileDialog::Detail);
	// Have the generator save
	if (dialog.exec())
	{
		QStringList qfilename = dialog.selectedFiles();
		std::string filename = qfilename.begin()->toStdString();

		// Force a .xml file extension here
		boost::filesystem::path pfile(filename);
		std::string ext = pfile.extension().string();
		if (ext != ".xml" && ext != ".bz2" && ext != ".gz")
			filename.append(".xml");

		rtmath::serialization::write<rtmath::ddscat::ddParGenerator>(_gen,filename);
	}
}

void frmMain::import()
{
	QFileDialog dialog(this);
	dialog.setFileMode(QFileDialog::ExistingFile);
	QStringList fltrs;
	fltrs << "PAR files (*.par)"
		<< "All Files (*.*)";

	dialog.setNameFilters(fltrs);
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
	{
		std::string sver = ui.cmbDdver->currentText().toStdString();
		if (sver == "7.0") 
		{
			_gen.ddscatVer = 70;
			_gen.base.version(70);
		}
		if (sver == "7.2")
		{
			_gen.base.version(72);
			_gen.ddscatVer = 72;
		}
	}
	//_gen.strPreCmds; // TODO
	//_gen.strPostCdms; // TODO
	//_gen.baseParFile = getValText<std::string>(ui.txtBaseFile);
	
	// shape stuff
	//_gen._shapeBase; // TODO
	// reg stuff
	_gen.compressResults = ui.chkCompress->isChecked();
	_gen.genIndivScripts = ui.chkGenIndivRunScripts->isChecked();
	_gen.genMassScript = ui.chkGenMassRunScript->isChecked();
	_gen.shapeStats = ui.chkGenShapeStats->isChecked();
	_gen.registerDatabase = ui.chkDatabaseRegister->isChecked();
	_gen.doExport = ui.chkDoExport->isChecked();
	_gen.exportLoc = getValText<std::string>(ui.txtExportDir);
	//_gen.exportLoc = ui.txtExportDir->text().toStdString();
	
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

		QTreeWidgetItem *i = ui.treeGlobals->topLevelItem(0);
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

	// Target properties
	// Need to use shape matching / casting function
	{
		_gen.shapes.clear();
		for (auto it = _targets.begin(); it != _targets.end(); it++)
		{
			// Take object, and use promote member function to get the correct object type
			boost::shared_ptr<rtmath::ddscat::shapeModifiable> casted(it->second->promote());

			// Then insert the object into _gen.shapes
			_gen.shapes.insert(casted);
		}
	}

	// Rotations
	{
		_gen.rots.clear();
		QTreeWidgetItem *i = ui.treeRots->topLevelItem(0);
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

			i = ui.treeRots->itemBelow(i);
		}
	}

	// Scattering angles
	{
		size_t n = 0;

		QTreeWidgetItem *i = ui.treeScattAngles->topLevelItem(0);
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
	
	ui.txtRunName->setText(QString::fromStdString(_gen.name));
	ui.txtDescription->setPlainText(QString::fromStdString(_gen.description));
	ui.txtOutLocation->setText(QString::fromStdString(_gen.outLocation));

	size_t ver = _gen.ddscatVer; // _gen.base.version() is not set on read. Only used on write.
	{
		// The indices will change as the program develops
		// These trigger ddverChanged
		if (ver == 70) ui.cmbDdver->setCurrentIndex(1);
		if (ver == 72) ui.cmbDdver->setCurrentIndex(0);
	}
	//_gen.ddscatVer;
	//_gen.strPreCmds; // TODO
	//_gen.strPostCdms; // TODO
	//ui.txtBaseFile->setText(QString::fromStdString(_gen.baseParFile));

	// shape stuff
	//_gen._shapeBase; // TODO
	// reg stuff
	ui.chkCompress->setChecked( _gen.compressResults );
	ui.chkGenIndivRunScripts->setChecked( _gen.genIndivScripts );
	ui.chkGenMassRunScript->setChecked( _gen.genMassScript );
	ui.chkGenShapeStats->setChecked( _gen.shapeStats );
	ui.chkDatabaseRegister->setChecked( _gen.registerDatabase );
	ui.chkDoExport->setChecked( _gen.doExport );
	ui.txtExportDir->setText(QString::fromStdString(_gen.exportLoc));
	ui.txtExportDir->setEnabled( _gen.doExport );

	ui.chkDoTorques->setChecked(_gen.base.doTorques());

	// For combo boxes, map from string to id number for selection
	{
		std::string s;

		_gen.base.getSolnMeth(s);
		if (s == "PBCGS2") ui.cmbSolnMeth->setCurrentIndex(0);
		if (s == "PBCGST") ui.cmbSolnMeth->setCurrentIndex(1);
		if (s == "PETRKP") ui.cmbSolnMeth->setCurrentIndex(2);

		_gen.base.getFFTsolver(s);
		if (s == "GPFAFT") ui.cmbFFTsolver->setCurrentIndex(0);
		if (s == "FFTMKL") ui.cmbFFTsolver->setCurrentIndex(1);

		_gen.base.getCalpha(s);
		if (s == "GKDLDR") ui.cmbCALPHA->setCurrentIndex(0);
		if (s == "LATTDIR") ui.cmbCALPHA->setCurrentIndex(1);

		_gen.base.getBinning(s);
		if (s == "NOTBIN") ui.cmbBinning->setCurrentIndex(0);
		if (s == "ORIBIN") ui.cmbBinning->setCurrentIndex(1);
		if (s == "ALLBIN") ui.cmbBinning->setCurrentIndex(2);
	}

	ui.txtImem1->setText(QString::number(_gen.base.Imem(0)));
	ui.txtImem2->setText(QString::number(_gen.base.Imem(1)));
	ui.txtImem3->setText(QString::number(_gen.base.Imem(2)));

	if (ver >= 72)
	{
		ui.chkNearfield->setChecked(_gen.base.doNearField());
		ui.txtNear1->setText(QString::number(_gen.base.near(0)));
		ui.txtNear2->setText(QString::number(_gen.base.near(1)));
		ui.txtNear3->setText(QString::number(_gen.base.near(2)));
		ui.txtNear4->setText(QString::number(_gen.base.near(3)));
		ui.txtNear5->setText(QString::number(_gen.base.near(4)));
		ui.txtNear6->setText(QString::number(_gen.base.near(5)));
	} else {
		ui.chkNearfield->setChecked(false);
		ui.txtNear1->setText("");
		ui.txtNear2->setText("");
		ui.txtNear3->setText("");
		ui.txtNear4->setText("");
		ui.txtNear5->setText("");
		ui.txtNear6->setText("");
	}

	ui.txtMaxTol->setText(QString::number(_gen.base.maxTol()));
	if (ver >= 72)
		ui.txtMaxIter->setText(QString::number(_gen.base.maxIter()));
	else
		ui.txtMaxIter->setText("");
	ui.txtGamma->setText(QString::number(_gen.base.gamma()));
	ui.txtETASCA->setText(QString::number(_gen.base.etasca()));
	if (ver >= 72)
		ui.txtNAMBIENT->setText(QString::number(_gen.base.nAmbient()));
	else
		ui.txtNAMBIENT->setText("");
	ui.chkWriteSca->setChecked(_gen.base.writeSca());

	// Global properties
	{
		ui.treeGlobals->clear();
		for (auto it = _gen.shapeConstraints.begin(); it != _gen.shapeConstraints.end(); it++)
		{
			QTreeWidgetItem *nitem = new QTreeWidgetItem();
			nitem->setText(0, QString::fromStdString(it->first));
			std::string srt;
			it->second->pset.getShort(srt);
			nitem->setText(1, QString::fromStdString(srt));
			nitem->setText(2, QString::fromStdString(it->second->units));
			ui.treeGlobals->addTopLevelItem(nitem);
		}
	}

	// Target properties
	{
		ui.treeTypes->clear();
		_targets.clear();
		_targetCounter = 0;
		for (auto it = _gen.shapes.begin(); it != _gen.shapes.end(); it++)
		{
			// Cast will demote back to shapeModifiable. This is desired.
			boost::shared_ptr<rtmath::ddscat::shapeModifiable> casted
				(new rtmath::ddscat::shapeModifiable);
			casted->shapeConstraints = (*it)->shapeConstraints;

			QTreeWidgetItem *nitem = new QTreeWidgetItem();
			_targetCounter++;
			nitem->setText(0, QString::number(_targetCounter));

			_targets.insert(std::pair<QTreeWidgetItem*, boost::shared_ptr<rtmath::ddscat::shapeModifiable> >
				(nitem, casted));
			ui.treeTypes->addTopLevelItem(nitem);
		}
	}

	// Rotations
	{
		ui.treeRots->clear();
		for (auto it = _gen.rots.begin(); it != _gen.rots.end(); it++)
		{
			QTreeWidgetItem *nitem = new QTreeWidgetItem();
			nitem->setText(0, QString::number((*it)->bMin()));
			nitem->setText(1, QString::number((*it)->bMax()));
			nitem->setText(2, QString::number((*it)->bN()));
			nitem->setText(3, QString::number((*it)->tMin()));
			nitem->setText(4, QString::number((*it)->tMax()));
			nitem->setText(5, QString::number((*it)->tN()));
			nitem->setText(6, QString::number((*it)->pMin()));
			nitem->setText(7, QString::number((*it)->pMax()));
			nitem->setText(8, QString::number((*it)->pN()));
			ui.treeRots->addTopLevelItem(nitem);
		}
	}

	// Scattering angles
	{
		ui.treeScattAngles->clear();
		for (size_t i=1; i<= _gen.base.numPlanes(); i++)
		{
			double phi, thetan_min, thetan_max, dtheta;
			_gen.base.getPlane(i,phi,thetan_min,thetan_max,dtheta);

			QTreeWidgetItem *nitem = new QTreeWidgetItem();
			nitem->setText(0, QString::number(phi));
			nitem->setText(1, QString::number(thetan_min));
			nitem->setText(2, QString::number(thetan_max));
			nitem->setText(3, QString::number(dtheta));
			ui.treeScattAngles->addTopLevelItem(nitem);
		}
	}
}

