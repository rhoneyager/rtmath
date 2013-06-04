#include "frmmain.h"
#include <boost/lexical_cast.hpp>
#include <string>
#include <complex>
#include <tmatrix/tmatrix.h>
#include "../../rtmath/rtmath/mie/mie-phaseFunc.h"
#include "../../rtmath/rtmath/mie/mie-Scalc.h"
#include "../../rtmath/rtmath/mie/mie-Qcalc.h"

#include <QMessageBox>
#include <QStandardItemModel>

frmMain::frmMain(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	import();
}

frmMain::~frmMain() {}

void frmMain::import()
{
	if (!tp) tp = tmatrix::tmatrixParams::create(
		10,1,6.283185,1.33,0,1,-1,0.001,2);
	
	auto set = [&](QLineEdit* t, double val)
	{
		t->setText( QString::number(val) );
	};

	set(ui.tAXI, tp->axi);
	set(ui.tRAT, tp->rat);
	set(ui.tLAM, tp->lam);
	set(ui.tMRR, tp->m.real());
	set(ui.tMRI, tp->m.imag());
	set(ui.tEPS, tp->eps);
	set(ui.tDDELT, tp->ddelt);
	set(ui.tNDGS, (double) tp->ndgs);
	set(ui.tNP, (double) tp->np);

	set(ui.tALPHA, 0);
	set(ui.tBETA, 0);
	set(ui.tTHET, 180);
	set(ui.tTHET0, 0);
	set(ui.tPHI, 0);
	set(ui.tPHI0, 0);
}

void frmMain::commit()
{
	// Take the values in the fields, commit and 
	// tabulate the results.
	double axi, rat, lam, mrr, mri, eps, ddelt, alpha;
	double beta, thet0, thet, phi0, phi;
	boost::int32_t np, ndgs;

	axi = ui.tAXI->text().toDouble();
	rat = ui.tRAT->text().toDouble();
	lam = ui.tLAM->text().toDouble();
	mri = ui.tMRI->text().toDouble();
	mrr = ui.tMRR->text().toDouble();
	eps = ui.tEPS->text().toDouble();
	ddelt = ui.tDDELT->text().toDouble();
	alpha = ui.tALPHA->text().toDouble();
	beta = ui.tBETA->text().toDouble();
	thet0 = ui.tTHET0->text().toDouble();
	thet = ui.tTHET->text().toDouble();
	phi0 = ui.tPHI0->text().toDouble();
	phi = ui.tPHI->text().toDouble();

	np = ui.tNP->text().toInt();
	ndgs = ui.tNDGS->text().toInt();

	tp = tmatrix::tmatrixParams::create(
		axi,rat,lam,mrr,mri,eps,np,ddelt,ndgs);
	to = tmatrix::OriTmatrix::calc(tp,alpha,beta);
	ta = tmatrix::OriAngleRes::calc(to,thet,thet0,phi,phi0);

	double tBksc, mBksc;
	double tQsca, tQext, tQabs;
	double mQsca, mQext, mQabs, mG;

	using namespace std;
	using boost::lexical_cast;

	complex<double> m(mrr,mri);
	const double pi = boost::math::constants::pi<double>();
	double x = 2. * pi * axi / lam;
	mie::miePhaseFunc pfm(x,m);
	mie::Scalc sm(m,x);
	auto mres = pfm.eval(abs(thet-thet0));
	double mSnn[4][4], mBnn[4][4];
	complex<double> mS[4],mB[4];
	sm.calc(cos(abs(thet-thet0)*pi/180.),mSnn,mS);
	sm.calc(cos(pi),mBnn,mB);
	mie::Qcalc mq(m);
	mq.calc(x,mQext,mQsca,mQabs,mG);
	// k is provided indirectly by the t-matrix
	// spectroscopic wavenumber = 1/wvlen, and angular wavenumber is 2pi/wvlen
	// we want k, which is the angular wavenumber
	double k = 2. * pi / lam;

	mBksc = (mBnn[0][0] / (k*k))  / (pi * axi*axi);


auto setM = [&](size_t row, size_t col, const std::string &val)
	{
		QTableWidgetItem *item = new QTableWidgetItem(QString::fromStdString(val));
		ui.mie->setItem(row,col,item);
	};
	for (size_t i=0;i<4;i++)
		setM(1+(i/2),1+(i%2),lexical_cast<string>(mS[i])); // ta->getS(i/2,i%2)));

	for (size_t i=0; i<16; i++)
		setM(4+(i/4),1+(i%4),lexical_cast<string>(mSnn[i/4][i%4])); //ta->getP(i/4,i%4)));

	setM(9,0,lexical_cast<string>(mQsca));
	setM(9,1,lexical_cast<string>(mQext));
	setM(9,2,lexical_cast<string>(mQabs));
	setM(9,3,lexical_cast<string>(mBksc));
	setM(9,4,lexical_cast<string>(mQsca/mQext));



	tQsca = to->qsca;
	tQext = to->qext;
	tQabs = tQext + tQsca;

	tBksc = tmatrix::getDifferentialBackscatterCrossSectionUnpol(to) / (pi * axi*axi);

	auto set = [&](size_t row, size_t col, const std::string &val)
	{
		QTableWidgetItem *item = new QTableWidgetItem(QString::fromStdString(val));
		ui.tm->setItem(row,col,item);
	};
	
	// Set tmatrix

	for (size_t i=0;i<4;i++)
		set(1+(i/2),1+(i%2),lexical_cast<string>(ta->getS(i/2,i%2)));

	for (size_t i=0; i<16; i++)
		set(4+(i/4),1+(i%4),lexical_cast<string>(ta->getP(i/4,i%4)));

	set(9,0,lexical_cast<string>(tQsca));
	set(9,1,lexical_cast<string>(tQext));
	set(9,2,lexical_cast<string>(tQabs));
	set(9,3,lexical_cast<string>(tBksc));
	set(9,4,lexical_cast<string>(tQsca/tQext));


}

