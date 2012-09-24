#include <complex>
#include <cmath>
#include <rtmath/mie/mie-phaseFunc.h>
#include <rtmath/mie/mie-Scalc.h>
#include <rtmath/matrixop.h>
#include <rtmath/units.h>
#include <tmatrix/tmatrix.h>

#include <boost/lexical_cast.hpp>
#include <boost/math/constants/constants.hpp>

#include "ui_frmMain.h"
#include "frmMain.h"

frmMain::frmMain()
	: ui(this)
{
}

frmMain::~frmMain()
{
}

void frmMain::exec()
{
	ui.wndMain->itemFocus(0);
	ui.wndMain->focus();
	textChanged(nullptr);
	ui.wndMain->beginMessageLoop();
}

bool frmMain::textChanged(const rycurses::ncHasText *item)
{
	// Well, any of the text boxes have changes, so let's 
	// update the calculations

	// Convert the textboxes into numbers, but only if the conversion
	// is possible. bad_lexical_cast is a bad idea here.
	try {
		// Note that mie and tmatrix code use different 
		// angle units. tmatrix uses degrees. mie uses radians.
		using namespace std;
		using namespace boost;
		tmatrix::tmatrix run;
		run.vars.AXI = lexical_cast<double>(ui.txtAXI->text);
		if (run.vars.AXI == 0) return false;
		run.vars.RAT = lexical_cast<double>(ui.txtRAT->text);
		run.vars.LAM = lexical_cast<double>(ui.txtLAM->text);
		if (!run.vars.LAM) return false;
		run.vars.MRR = lexical_cast<double>(ui.txtMRR->text);
		if (abs(run.vars.MRR - 1.0) < 0.05) return false;
		if (!run.vars.MRR) return false;
		run.vars.MRI = lexical_cast<double>(ui.txtMRI->text);
		run.vars.EPS = lexical_cast<double>(ui.txtEPS->text);
		if (!run.vars.EPS) return false;
		run.vars.DDELT = lexical_cast<double>(ui.txtDDELT->text);
		if (!run.vars.DDELT) return false;
		run.vars.ALPHA = lexical_cast<double>(ui.txtALPHA->text);
		run.vars.BETA = lexical_cast<double>(ui.txtBETA->text);
		run.vars.THET0 = lexical_cast<double>(ui.txtTHET0->text);
		run.vars.THET = lexical_cast<double>(ui.txtTHET->text);
		run.vars.PHI0 = lexical_cast<double>(ui.txtPHI0->text);
		run.vars.PHI = lexical_cast<double>(ui.txtPHI->text);
		run.vars.NP = lexical_cast<int>(ui.txtNP->text);
		if (!run.vars.NP) return false;
		run.vars.NDGS = lexical_cast<int>(ui.txtNDGS->text);
		if (run.vars.NDGS < 2) return false;

		run.run();

		ui.lblXV->text = lexical_cast<string>( 1 );
		for (size_t i=0; i<4;i++)
		{
			ui.lblTSi[i]->text = lexical_cast<string>( 
				run.outs.S[i]);
			for (size_t j=0;j<4;j++)
			{
				double num = run.outs.P[i][j];
				if (isfinite(num) && 0)
				{
					ostringstream val;
					val.precision(8);
					val << run.outs.P[i][j];
					string s;
					s = val.str();
					ui.lblTPi[(4*i)+j]->text = s;
				}
				ui.lblTPi[(4*i)+j]->text = lexical_cast<string>(num);
			}
		}

		// Figure out the size parameter
		const double pi = boost::math::constants::pi<double>();
		double x = 2.0 * pi * run.vars.AXI / run.vars.LAM;
		ui.lblXV->text = lexical_cast<string>(x);

		// Also do the mie calculation
		complex<double> m(run.vars.MRR,run.vars.MRI);
		mie::miePhaseFunc pfm(x,m);
		mie::Scalc sm(m,x);
		auto mres = pfm.eval(abs(run.vars.THET - run.vars.THET0));
		double mSnn[4][4];
		complex<double> mS[4];
		sm.calc(cos(abs(run.vars.THET - run.vars.THET0)*pi/180.), mSnn, mS);

		// mres is a std::shared_ptr<matrixop>
		for (size_t i=0; i<4; i++)
		{
			ui.lblMSi[i]->text = lexical_cast<string>(
				mS[i]);
			for (size_t j=0;j<4;j++)
			{
				double num = mres->get(2,i,j);
				if (isfinite(num) && 0)
				{
					ostringstream val;
					val.precision(8);
					val << run.outs.P[i][j];
					string s;
					s = val.str();
					ui.lblMPi[(4*i)+j]->text = s;
				}
				ui.lblMPi[(4*i)+j]->text = lexical_cast<string>(num);
			}
		}
	}
	catch (std::bad_cast &e)
	{
		return false;
	}
	return true;
}

