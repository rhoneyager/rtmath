#include "frmmain.h"
#include <string>
#include <set>
#include <vector>
#include <map>

#include <QDir>
#include <QFileDialog>
#include <QMessageBox>

#include <boost/filesystem.hpp>
/*
#include "../../rtmath/rtmath/serialization.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
*/
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
	double T = getValText<double>(ui.txtTemp);

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
	boost::filesystem::path pPar(defaultPar);
	if (!boost::filesystem::exists(pPar) && defaultPar.size())
	{
		QMessageBox::critical(this, tr("tmatrix-convert-from-ddscat"), 
			tr("Default par file does not exist!"));
		return;
	}

	boost::filesystem::path pDir(baseDir);
	if (!boost::filesystem::exists(pDir))
	{
		QMessageBox::critical(this, tr("tmatrix-convert-from-ddscat"), 
			tr("Base directory does not exist!"));
		return;
	}

	// Convert the combo boxes into real options


	// Finally, begin forming the iteration

	// Iterate over all matching files and perform the t-matrix generation

}
