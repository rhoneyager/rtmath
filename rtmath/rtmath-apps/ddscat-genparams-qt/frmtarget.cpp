#include "stdafx.h"
#include "frmtarget.h"
#include <QFileDialog>
#include <QMenu>

using namespace rtmath::ddscat;

frmTarget::frmTarget(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	ui.setupUi(this);
}

frmTarget::~frmTarget()
{

}

void frmTarget::processOK()
{
	toShapeModifiable();
	this->close();
}

void frmTarget::targetTypeChanged()
{
	if (ui.cmbTargetType->currentIndex() == 0)
	{
		ui.treeShapedat->setEnabled(true);
	} else {
		ui.treeShapedat->setEnabled(false);
	}
}

void frmTarget::dimReInsChanged()
{
}

void frmTarget::setTarget(boost::shared_ptr<rtmath::ddscat::shapeModifiable> tgt)
{
	// the tgt entry is not a particular shape class. The entry gets recreated as one of the 
	// appropriate type through a casting function.

	this->tgt = tgt;
	fromShapeModifiable();
}

void frmTarget::editTreeItem(QTreeWidgetItem* item,int column)
{
	item->setFlags(item->flags() | Qt::ItemIsEditable);
	item->treeWidget()->editItem(item,column);
	//ui.treeFreqs->editItem(item, column);
}

void frmTarget::menuTargetProps(const QPoint &p)
{
	QPoint pos = ui.treeConstraints->mapToGlobal(p);
	QMenu mnuF;
	QAction *mAdd, *mRemove;
	mAdd = mnuF.addAction("&Add");
	mRemove = mnuF.addAction("&Remove");

	QList<QTreeWidgetItem *> selFreqs = ui.treeConstraints->selectedItems();
	(selFreqs.size())? mRemove->setEnabled(true) : mRemove->setEnabled(false);

	QAction *selected = mnuF.exec(pos);
	if (selected)
	{
		// Will manipulate items and their mapped frequency entries (text strings)
		if (selected == mAdd)
		{
			QTreeWidgetItem *nitem = new QTreeWidgetItem();
			nitem->setText(0, "aeff");
			nitem->setText(1, "100");
			nitem->setText(2, "um");
			ui.treeConstraints->addTopLevelItem(nitem);
		}
		if (selected == mRemove)
		{
			// Drop the selected items
			for (auto it = selFreqs.begin(); it != selFreqs.end(); it++)
				delete *it;
		}
	}
}

void frmTarget::menuShapeDatProps(const QPoint &p)
{
	QPoint pos = ui.treeShapedat->mapToGlobal(p);
	QMenu mnuF;
	QAction *mAdd, *mAddb, *mRemove;
	mAdd = mnuF.addAction("&Add");
	mAddb = mnuF.addAction("&Add (alternate)");
	mRemove = mnuF.addAction("&Remove");

	QList<QTreeWidgetItem *> selFreqs = ui.treeShapedat->selectedItems();
	(selFreqs.size())? mRemove->setEnabled(true) : mRemove->setEnabled(false);

	QAction *selected = mnuF.exec(pos);
	if (selected)
	{
		// Will manipulate items and their mapped frequency entries (text strings)
		if (selected == mAdd)
		{
			
			QFileDialog dialog(this);
			dialog.setFileMode(QFileDialog::ExistingFile);
			QStringList fltrs;
			fltrs << "Shape Files (*.dat *.shp)"
				<< "All Files (*.*)";

			dialog.setNameFilters(fltrs);
			dialog.setViewMode(QFileDialog::Detail);
	
			if (dialog.exec())
			{
				QStringList qfilename;
				qfilename = dialog.selectedFiles();

				for (auto it = qfilename.begin(); it != qfilename.end(); it++)
				{
					//std::string filename = it->toStdString();
					QTreeWidgetItem *nitem = new QTreeWidgetItem();
					nitem->setText(0, *it);
					ui.treeShapedat->addTopLevelItem(nitem);
				}
			}
		}
		if (selected == mAddb)
		{
			QTreeWidgetItem *nitem = new QTreeWidgetItem();
			nitem->setText(0, "shape.dat");
			ui.treeShapedat->addTopLevelItem(nitem);
		}
		if (selected == mRemove)
		{
			// Drop the selected items
			for (auto it = selFreqs.begin(); it != selFreqs.end(); it++)
				delete *it;
		}
	}
}

void frmTarget::fromShapeModifiable()
{
	ui.treeShapedat->clear();
	ui.cmbTargetType->setCurrentIndex(0);
	ui.treeConstraints->clear();

	{
		ui.treeShapedat->clear();

		std::string shp;
		auto it = tgt->shapeConstraints.find("CSHAPE");
		if (it != tgt->shapeConstraints.end()) shp = it->second->units;
			
		// TODO: enum these
		if (shp == "FROM_FILE")
			ui.cmbTargetType->setCurrentIndex(0);
		if (shp == "ELLIPSOID")
			ui.cmbTargetType->setCurrentIndex(1);
		// TODO: change these id strings
		if (shp == "HEX_PLATE")
			ui.cmbTargetType->setCurrentIndex(2);
		if (shp == "REC_PRISM")
			ui.cmbTargetType->setCurrentIndex(3);
		if (shp == "NEEDLE")
			ui.cmbTargetType->setCurrentIndex(4);
		if (shp == "TMATRIX_ELLIPSOID")
			ui.cmbTargetType->setCurrentIndex(5);
		if (shp == "TMATRIX_CYLINDER")
			ui.cmbTargetType->setCurrentIndex(6);

		targetTypeChanged();
	}

	// Now, process the other shapeConstraints
	for (auto it = tgt->shapeConstraints.begin(); it != tgt->shapeConstraints.end(); it++)
	{
		// "source_filename"
		if (it->first == "source_filename")
		{
			QTreeWidgetItem *nitem = new QTreeWidgetItem();
			nitem->setText(0, QString::fromStdString(it->second->units));
			ui.treeShapedat->addTopLevelItem(nitem);
		} else {
			QTreeWidgetItem *nitem = new QTreeWidgetItem();
			nitem->setText(0, QString::fromStdString(it->first));
			std::string srt;
			it->second->pset.getShort(srt);
			nitem->setText(1, QString::fromStdString(srt));
			nitem->setText(2, QString::fromStdString(it->second->units));
		}
	}
}

void frmTarget::toShapeModifiable()
{
	tgt->shapeConstraints.clear();

	std::string shp;
	switch (ui.cmbTargetType->currentIndex())
	{
		// TODO enum these
	case 0:
		shp = "FROM_FILE";
		break;
	case 1:
		shp = "ELLIPSOID";
		break;
	default:
		shp = "";
	}
	
	tgt->addConstraint( shapeConstraint::create("CSHAPE", 0, shp) );

	// Add any possible shape.dat files
	//for (auto it = 
	{
		QTreeWidgetItem *i = ui.treeShapedat->takeTopLevelItem(0);
		while (i)
		{
			std::string fname;
			fname = i->text(0).toStdString();

			using namespace rtmath::ddscat;
			tgt->addConstraint(shapeConstraint::create("source_filename", 0, fname));

			i = ui.treeShapedat->itemBelow(i);
		}
	}

	// Add everything else
	{
		QTreeWidgetItem *i = ui.treeConstraints->takeTopLevelItem(0);
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
			tgt->addConstraint(shapeConstraint::create(vname, range, units));

			i = ui.treeConstraints->itemBelow(i);
		}
	}
}

