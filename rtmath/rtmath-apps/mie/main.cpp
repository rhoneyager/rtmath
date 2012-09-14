#pragma warning( disable : 4996 ) // Dumb boost uuid warning
#pragma warning( disable : 4800 ) // forcing non-bool type to true or false
#pragma warning( disable : 4521 ) // multiple copy constructors in some PCL stuff
#pragma warning( disable : 4244 ) // warning C4244: '=' : conversion from 'double' to 'float', possible loss of data in FLANN
#include <iostream>
#include <complex>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <memory>
#include <string>
#include <algorithm>
#include <boost/filesystem.hpp> // used for location of output of netcdf
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>
#include <boost/math/constants/constants.hpp>


#include "../../rtmath/rtmath/ROOTlink.h"

#include "../../rtmath/rtmath/mie/mie-Scalc.h"
#include "../../rtmath/rtmath/mie/mie-phaseFunc.h"
#include "../../rtmath/rtmath/error/error.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/matrixop.h"
#include "../../deps/tmatrix/src/headers/tmatrix.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-mie" << endl;
		rtmath::debug::appEntry(argc, argv);

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("frequencies", 0);
		p.add("sizes", 1);
		p.add("output-prefix", 2);

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("sizeparams,x", po::value< string >(), "input size parameters")
			("thetas", po::value< string >()->default_value("0:1:180"), "scattering angles to consider")
			("tmatrix", "Do tmatrix run")
			("mie", "Do mie run")
			("Mreal", po::value<double>()->default_value(1.33), "Real refractive index")
			("Mimag", po::value<double>()->default_value(0), "Imaginary refractive index")
			("output-prefix,o", po::value<string>()->default_value("output"), "Specify output filename prefix");

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);    

		if (vm.count("help") || argc == 1 || !vm.count("sizeparams") ) {
			cerr << desc << "\n";
			return 1;
		}


		// Take ddscat name or path from argv, and attempt to load the files
		string sX = vm["sizeparams"].as<string>();
		string sThetas = vm["thetas"].as<string>();

		paramSet<double> Xs(sX);
		paramSet<double> thetas(sThetas);
		bool doMie = true;
		bool doTm = false;
		complex<double> M( vm["Mreal"].as<double>(), vm["Mimag"].as<double>());
		if (vm.count("mie")) doMie = true;
		if (vm.count("tmatrix")) doTm = true;

		string outprefix = vm["output-prefix"].as<string>();

		boost::shared_array<double> Othetas(new double[thetas.size()]);
		std::copy(thetas.begin(), thetas.end(), Othetas.get());
		

		for (auto X = Xs.begin(); X != Xs.end(); ++X)
		{
			mie::Scalc mS(M,*X);
			mie::miePhaseFunc mP(*X,M);
			::tmatrix::tmatrix tm;
			tm.vars.LAM = 2.0 * boost::math::constants::pi<double>();
			tm.vars.AXI = *X;
			tm.vars.THET0 = 0;
			tm.vars.MRR = M.real();
			tm.vars.MRI = M.imag();
			tm.vars.EPS = 1.0;

			ostringstream op;
			op << outprefix << "-X-" << *X << "-";
			string fP(op.str());
			fP.append("P.png");
			string fS(op.str());
			fS.append("S.png");

			// Standard vector constructor would lead to mempry problems
			vector<boost::shared_array<double> > 
				miePnn, mieSnn, tmPnn, mieSr, mieSi, tmSr, tmSi;
			miePnn.reserve(16);
			mieSnn.reserve(16);
			tmPnn.reserve(16);
			mieSr.reserve(4);
			mieSi.reserve(4);
			tmSr.reserve(4);
			tmSi.reserve(4);
			for (size_t i=0; i<16;i++)
			{
				miePnn.push_back(boost::shared_array<double>(new double[thetas.size()]));
				mieSnn.push_back(boost::shared_array<double>(new double[thetas.size()]));
				tmPnn.push_back(boost::shared_array<double>(new double[thetas.size()]));
			}
			for (size_t i=0;i<4;i++)
			{
				mieSr.push_back(boost::shared_array<double>(new double[thetas.size()]));
				mieSi.push_back(boost::shared_array<double>(new double[thetas.size()]));
				tmSr.push_back(boost::shared_array<double>(new double[thetas.size()]));
				tmSi.push_back(boost::shared_array<double>(new double[thetas.size()]));
			}

			size_t i=0;
			for (auto theta = thetas.begin(); theta != thetas.end(); ++theta, ++i)
			{
				double thetar = *theta * boost::math::constants::pi<double>() / 180.0;
				double mu = cos(thetar);

				double mSnn[4][4], mPnn[4][4];
				complex<double> mSn[4];
				mS.calc(mu,mSnn,mSn);
				auto mmPnn = mP.eval(thetar);
				mmPnn->toDoubleArray(&mPnn[0][0]);
				
				tm.vars.THET = *theta;
				if (doTm)
					tm.run();

				// Stupid, but it really helps ROOT plotting

				mieSr[0][i]= mSn[0].real();
				mieSr[1][i]= mSn[1].real();
				mieSr[2][i]= mSn[2].real();
				mieSr[3][i]= mSn[3].real();
				mieSi[0][i]= mSn[0].imag();
				mieSi[1][i]= mSn[1].imag();
				mieSi[2][i]= mSn[2].imag();
				mieSi[3][i]= mSn[3].imag();

				tmSr[0][i]= tm.outs.S[0].real();
				tmSr[1][i]= tm.outs.S[1].real();
				tmSr[2][i]= tm.outs.S[2].real();
				tmSr[3][i]= tm.outs.S[3].real();
				tmSi[0][i]= tm.outs.S[0].imag();
				tmSi[1][i]= tm.outs.S[1].imag();
				tmSi[2][i]= tm.outs.S[2].imag();
				tmSi[3][i]= tm.outs.S[3].imag();

				mieSnn[0][i] = mSnn[0][0];
				mieSnn[1][i] = mSnn[0][1];
				mieSnn[2][i] = mSnn[0][2];
				mieSnn[3][i] = mSnn[0][3];
				mieSnn[4][i] = mSnn[1][0];
				mieSnn[5][i] = mSnn[1][1];
				mieSnn[6][i] = mSnn[1][2];
				mieSnn[7][i] = mSnn[1][3];
				mieSnn[8][i] = mSnn[2][0];
				mieSnn[9][i] = mSnn[2][1];
				mieSnn[10][i] = mSnn[2][2];
				mieSnn[11][i] = mSnn[2][3];
				mieSnn[12][i] = mSnn[3][0];
				mieSnn[13][i] = mSnn[3][1];
				mieSnn[14][i] = mSnn[3][2];
				mieSnn[15][i] = mSnn[3][3];

				miePnn[0][i] = mPnn[0][0];
				miePnn[1][i] = mPnn[0][1];
				miePnn[2][i] = mPnn[0][2];
				miePnn[3][i] = mPnn[0][3];
				miePnn[4][i] = mPnn[1][0];
				miePnn[5][i] = mPnn[1][1];
				miePnn[6][i] = mPnn[1][2];
				miePnn[7][i] = mPnn[1][3];
				miePnn[8][i] = mPnn[2][0];
				miePnn[9][i] = mPnn[2][1];
				miePnn[10][i] = mPnn[2][2];
				miePnn[11][i] = mPnn[2][3];
				miePnn[12][i] = mPnn[3][0];
				miePnn[13][i] = mPnn[3][1];
				miePnn[14][i] = mPnn[3][2];
				miePnn[15][i] = mPnn[3][3];

				tmPnn[0][i] = tm.outs.P[0][0];
				tmPnn[1][i] = tm.outs.P[0][1];
				tmPnn[2][i] = tm.outs.P[0][2];
				tmPnn[3][i] = tm.outs.P[0][3];
				tmPnn[4][i] = tm.outs.P[1][0];
				tmPnn[5][i] = tm.outs.P[1][1];
				tmPnn[6][i] = tm.outs.P[1][2];
				tmPnn[7][i] = tm.outs.P[1][3];
				tmPnn[8][i] = tm.outs.P[2][0];
				tmPnn[9][i] = tm.outs.P[2][1];
				tmPnn[10][i] = tm.outs.P[2][2];
				tmPnn[11][i] = tm.outs.P[2][3];
				tmPnn[12][i] = tm.outs.P[3][0];
				tmPnn[13][i] = tm.outs.P[3][1];
				tmPnn[14][i] = tm.outs.P[3][2];
				tmPnn[15][i] = tm.outs.P[3][3];
			}
			
			boost::shared_ptr<TCanvas> tS(new TCanvas("S","S", 800*4,800*2));
			gStyle->SetPalette(1);
			tS->Divide(4,2);

			//size_t color = 1;

			vector<boost::shared_ptr<TGraph> > graphs;
			graphs.reserve(48);

			for (Int_t j=0; j<4; j++)
			{
				ostringstream rTitle, iTitle;
				rTitle << "Real S_{" << (j+1) << "}";
				iTitle << "Imaginary S_{" << (j+1) << "}";
				boost::shared_ptr<TGraph> gMr(new TGraph(thetas.size(), Othetas.get(), mieSr[j].get()));
				boost::shared_ptr<TGraph> gMi(new TGraph(thetas.size(), Othetas.get(), mieSi[j].get()));
				boost::shared_ptr<TGraph> gTr(new TGraph(thetas.size(), Othetas.get(), tmSr[j].get()));
				boost::shared_ptr<TGraph> gTi(new TGraph(thetas.size(), Othetas.get(), tmSi[j].get()));
				graphs.push_back(gMr);
				graphs.push_back(gMi);
				graphs.push_back(gTr);
				graphs.push_back(gTi);
				tS->cd(j+1);
				gMr->Draw("AC");
				gMr->SetLineColor(1);
				gMr->SetTitle(rTitle.str().c_str());
				gMr->GetXaxis()->SetTitle("#Theta");
				gMr->GetXaxis()->CenterTitle();
				gMr->GetYaxis()->SetTitle("S_{real}");
				gMr->GetYaxis()->CenterTitle();
				if (doTm)
					gTr->Draw("C");
				gTr->SetLineColor(2);
				tS->cd(5+j);
				gMi->Draw("AC");
				gMi->SetLineColor(1);
				gMi->SetTitle(iTitle.str().c_str());
				gMi->GetXaxis()->SetTitle("#Theta");
				gMi->GetXaxis()->CenterTitle();
				gMi->GetYaxis()->SetTitle("S_{imag}");
				gMi->GetYaxis()->CenterTitle();
				if (doTm)
					gTi->Draw("C");
				gTi->SetLineColor(2);
			}
			tS->SaveAs(fS.c_str());

			boost::shared_ptr<TCanvas> tcP(new TCanvas("P","P", 800*4,800*4));
			tcP->Divide(4,4);

			for (size_t j=0; j<4; j++)
			{
				for (size_t k=0; k<4; k++)
				{
					ostringstream title;
					title << "P_{" << (j+1) << (k+1) << "}";
					boost::shared_ptr<TGraph> gP(new TGraph(thetas.size(), Othetas.get(), miePnn[(4*j)+k].get()));
					boost::shared_ptr<TGraph> tP(new TGraph(thetas.size(), Othetas.get(), tmPnn[(4*j)+k].get()));
					graphs.push_back(gP);
					graphs.push_back(tP);
					tcP->cd((4*j)+k+1);
					gP->Draw("AC");
					gP->SetLineColor(1);
					gP->SetTitle(title.str().c_str());
					gP->GetXaxis()->SetTitle("#Theta");
					gP->GetXaxis()->CenterTitle();
					gP->GetYaxis()->SetTitle(title.str().c_str());
					gP->GetYaxis()->CenterTitle();
					if (doTm)
						tP->Draw("C");
					tP->SetLineColor(2);
				}
			}
			tcP->SaveAs(fP.c_str());
		}
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		exit(1);
	}
	return 0;
}

