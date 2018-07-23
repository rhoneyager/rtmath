#include <string>
#include <iostream>
#include <fstream>
//#include <QMessageBox>
#include "frmmain.h"

frmmain::frmmain(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	ui.setupUi(this);
	thr.initt(&tm,&ui);
	QObject::connect(&thr, SIGNAL(finished()), this, SLOT(done()) );
	populate();
}

frmmain::~frmmain()
{
	thr.wait(); // Wait until thread completes execution
}

void frmmain::populate()
{
	// Take values from tmatrixInVars and populate fields
	ui.txtAXI->setText(QString::number(tm.vars.AXI));
	ui.txtRAT->setText(QString::number(tm.vars.RAT));
	ui.txtLAM->setText(QString::number(tm.vars.LAM));
	ui.txtMRR->setText(QString::number(tm.vars.MRR));
	ui.txtMRI->setText(QString::number(tm.vars.MRI));
	ui.txtEPS->setText(QString::number(tm.vars.EPS));
	ui.txtNP->setText(QString::number(tm.vars.NP));
	ui.txtDDELT->setText(QString::number(tm.vars.DDELT));
	ui.txtNDGS->setText(QString::number(tm.vars.NDGS));
	ui.txtALPHA->setText(QString::number(tm.vars.ALPHA));
	ui.txtBETA->setText(QString::number(tm.vars.BETA));
	ui.txtTHET0->setText(QString::number(tm.vars.THET0));
	ui.txtTHET->setText(QString::number(tm.vars.THET));
	ui.txtPHI0->setText(QString::number(tm.vars.PHI0));
	ui.txtPHI->setText(QString::number(tm.vars.PHI));
}

void frmmain::run()
{
	// Validate output
	
	// Convert to tmatrixInVars entries
	tm.vars.AXI = ui.txtAXI->text().toDouble();
	tm.vars.RAT = ui.txtRAT->text().toDouble();
	tm.vars.LAM = ui.txtLAM->text().toDouble();
	tm.vars.MRR = ui.txtMRR->text().toDouble();
	tm.vars.MRI = ui.txtMRI->text().toDouble();
	tm.vars.EPS = ui.txtEPS->text().toDouble();
	tm.vars.NP = ui.txtNP->text().toInt();
	tm.vars.DDELT = ui.txtDDELT->text().toDouble();
	tm.vars.NDGS = ui.txtNDGS->text().toInt();
	tm.vars.ALPHA = ui.txtALPHA->text().toDouble();
	tm.vars.BETA = ui.txtBETA->text().toDouble();
	tm.vars.THET0 = ui.txtTHET0->text().toDouble();
	tm.vars.THET = ui.txtTHET->text().toDouble();
	tm.vars.PHI0 = ui.txtPHI0->text().toDouble();
	tm.vars.PHI = ui.txtPHI->text().toDouble();

	// Put the run into a separate thread...
	ui.cmdRun->setEnabled(false);
	thr.start();
}

void frmmain::done()
{
	// Serialize the output to the target destination
	// Not using rtmath, as rtmath depends on this project
	// TODO: implement better serialization
	using namespace std;
	std::string fname = toUtf8(ui.txtOutput->text());
	ofstream out(fname.c_str());
	boost::archive::xml_oarchive oa(out);
	oa << BOOST_SERIALIZATION_NVP(tm.vars);
	oa << BOOST_SERIALIZATION_NVP(tm.outs);

	// Once finished, reenable cmdRun
	ui.cmdRun->setEnabled(true);
}

void tmExecThread::initt(tmatrix::tmatrix *tm, Ui::frmmainClass *ui)
{
	_tm = tm;
	_ui = ui;
}

void tmExecThread::run()
{
	_tm->run();
}


