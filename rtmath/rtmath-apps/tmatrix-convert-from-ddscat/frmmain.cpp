#include "frmmain.h"
#include <string>
#include <set>
#include <vector>
#include <map>

#include <QDir>
#include <QFileDialog>
#include <QMessageBox>

#include <boost/filesystem.hpp>

#include "converter.h"

#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/serialization.h"
#include "../../rtmath/rtmath/Serialization/shapestats_serialization.h"
//#include "../../rtmath/rtmath/ddscat/ddpar.h"
//#include "../../rtmath/rtmath/ddscat/shapefile.h"

template <>
double getValText(QLineEdit *src)
{
#ifndef RTMATH_OLD_QT
	return src->text().size() ? src->text().toDouble() : src->placeholderText().toDouble() ;
#else
	return src->text().toDouble();
#endif
}

template <>
std::string getValText(QLineEdit *src)
{
#ifndef RTMATH_OLD_QT
	return src->text().size() ? src->text().toStdString() : src->placeholderText().toStdString();
#else
	return src->text().toStdString();
#endif
}

frmMain::frmMain(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	ui.setupUi(this);
	/* Older Qt versions do not support placeHolderText (ahem, crystallus).
	 * To allow for compilation, while still having placeholerText on the 
	 * newer versions, insert the text definitions here. */
#ifndef RTMATH_OLD_QT
	ui.txtTemp->setPlaceholderText(QApplication::translate("frmMainClass", "263", 0, QApplication::UnicodeUTF8));
	ui.txtDiel->setPlaceholderText(QApplication::translate("frmMainClass", "0.85", 0, QApplication::UnicodeUTF8));
#else
	ui.txtTemp->setText(QApplication::translate("frmMainClass", "263", 0, QApplication::UnicodeUTF8));
	ui.txtDiel->setText(QApplication::translate("frmMainClass", "0.85", 0, QApplication::UnicodeUTF8));

#endif
}

frmMain::~frmMain()
{

}

void frmMain::findDefaultPar()
{
	// Open dialog box asking from where
	
	QFileDialog dialog(this);
	dialog.setFileMode(QFileDialog::ExistingFile);
	QStringList fltrs;
	fltrs << "ddscat.par (ddscat.par)"
		<< "All Files (*.*)";

	dialog.setNameFilters(fltrs);
	dialog.setViewMode(QFileDialog::Detail);
	
	if (dialog.exec())
	{
		QStringList qfilename;
		qfilename = dialog.selectedFiles();

		// Have generator load
		std::string filename = qfilename.begin()->toStdString();

		ui.txtDefaultPar->setText(QString::fromStdString(filename));
	}
}

void frmMain::findBaseDir()
{
	// Open dialog box asking from where
	QString path = QFileDialog::getExistingDirectory (this, tr("Select base directory"));
	if (path.size() == 0) return;
	ui.txtBaseDir->setText(path);

}

void frmMain::doGenerate()
{
	using namespace std;
	using namespace boost::filesystem;
	double T = getValText<double>(ui.txtTemp);
	double nu = getValText<double>(ui.txtDiel);

	string defaultPar = getValText<string>(ui.txtDefaultPar);
	string baseDir = getValText<string>(ui.txtBaseDir);

	if (!baseDir.size())
	{
		QMessageBox::critical(this, tr("tmatrix-convert-from-ddscat"), 
			tr("Must specify a base directory"));
		return;
	}

	if (!defaultPar.size())
	{
		// Try and find a default par file from the configuration

		QMessageBox::warning(this, tr("tmatrix-convert-from-ddscat"),
			tr("Using rtmath base par file when none is detected"));
	}

	// Verify file existence
	path pPar(defaultPar);
	if (!exists(pPar) && defaultPar.size())
	{
		QMessageBox::critical(this, tr("tmatrix-convert-from-ddscat"), 
			tr("Default par file does not exist!"));
		return;
	}

	path pDir(baseDir);
	if (!exists(pDir))
	{
		QMessageBox::critical(this, tr("tmatrix-convert-from-ddscat"), 
			tr("Base directory does not exist!"));
		return;
	}
	if (!is_directory(pDir))
	{
		QMessageBox::critical(this, tr("tmatrix-convert-from-ddscat"),
			tr("Need to specify a directory for base directory!"));
		return;
	}

	// Convert the combo boxes into real options
	string shapePattern = ui.cmbShapePattern->currentText().toStdString();
	string shapeMethod = ui.cmbShapeMeth->currentText().toStdString();
	string dielMethod = ui.cmbDiel->currentText().toStdString();
	string volFracMethod = ui.cmbVolFrac->currentText().toStdString();

	bool searchExt = false;
	bool searchFile = false;
	if (shapePattern.find('*') != string::npos)
	{
		searchExt = true;
		shapePattern = shapePattern.substr(1);
	} else {
		searchFile = true;
	}
	// Finally, begin forming the iteration

	// Iterate over all matching files and perform the t-matrix generation
	vector<path> v;
	copy(recursive_directory_iterator(pDir), recursive_directory_iterator(), back_inserter(v));
	for (auto it = v.begin(); it != v.end(); it++)
	{
		if (!is_regular_file(*it)) continue;
		// Check that this is a recognized shape.dat file
		path pBase, pFile, pCandPar;
		pFile = it->filename();
		pBase = it->parent_path();
		pCandPar = pBase / "ddscat.par";

		if (searchFile)
		{
			if (pFile.string() != shapePattern)
				continue;
		} else { // Search on extension
			if (pFile.extension().string() != shapePattern)
				continue;
		}

		// Only the desited shape files will make it to this point!
		string pDest = it->string(); 
		pDest.append("-tmatrix.xml");
		string pStats = it->string(); 
		pStats.append("-stats.xml");

		// If ddscat.par is in this dir, use it. Else, use default.
		if (!exists(pCandPar)) pCandPar = pPar;
		// Do the shape stats need to be generated?
		boost::shared_ptr<rtmath::ddscat::shapeFileStats> stats;
		if (!exists(path(pStats)))
		{
			stats = rtmath::ddscat::shapeFileStats::genStats(it->string(),pStats);
		} else {
			rtmath::ddscat::shapeFileStats s;
			rtmath::serialization::read<rtmath::ddscat::shapeFileStats>(s,it->string());
			stats = boost::shared_ptr<rtmath::ddscat::shapeFileStats>(
				new rtmath::ddscat::shapeFileStats(s));
		}

		// Everything else is handled by the converter
		fileconverter cnv;
		//cnv.setShapeFile(it->string());
		cnv.setStats(stats);
		cnv.setDDPARfile(pCandPar.string());

		//cnv.setShapePattern("");
		cnv.setShapeMethod(shapeMethod);
		cnv.setDielMethod(dielMethod,nu);
		cnv.setVolFracMethod(volFracMethod);
		cnv.setTemp(T);

		//ui.cmdGenerate->
		cnv.convert(pDest);
	}

}
