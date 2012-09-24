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
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>

#include "../../rtmath/rtmath/ROOTlink.h"
#include <TLegend.h>

#include "../../rtmath/rtmath/phaseFunc.h"
#include "../../rtmath/rtmath/mie/mie-Scalc.h"
#include "../../rtmath/rtmath/mie/mie-phaseFunc.h"
#include "../../rtmath/rtmath/mie/mie-Qcalc.h"
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
			tm.vars.RAT = 1;
			tm.vars.DDELT = 0.001;
			tm.vars.ALPHA = 0;
			tm.vars.BETA = 0;
			tm.vars.PHI0 = 0;
			tm.vars.PHI = 0;
			tm.vars.NP = -1;
			tm.vars.NDGS = 3;

			ostringstream op;
			op << outprefix << "-X-" << *X << "-";
			string fP(op.str());
			fP.append("P.png");
			string fS(op.str());
			fS.append("S.png");

			// Standard vector constructor would lead to mempry problems
			vector<boost::shared_array<double> > 
				miePnn, mieSnn, tmPnn, mieSr, mieSi, tmSr, tmSi,
				tmSrC, tmSiC, tmPnnC;
			miePnn.reserve(16);
			mieSnn.reserve(16);
			tmPnn.reserve(16);
			mieSr.reserve(4);
			mieSi.reserve(4);
			tmSr.reserve(4);
			tmSi.reserve(4);

			tmSrC.reserve(4);
			tmSiC.reserve(4);
			tmPnnC.reserve(16);

			using namespace boost::accumulators;
			vector<accumulator_set<double, stats<tag::min, tag::max> > > 
				accsP, accsSr;
			accsP.resize(16);
			accsSr.resize(4);
			for (size_t i=0; i<16;i++)
			{
				miePnn.push_back(boost::shared_array<double>(new double[thetas.size()]));
				mieSnn.push_back(boost::shared_array<double>(new double[thetas.size()]));
				tmPnn.push_back(boost::shared_array<double>(new double[thetas.size()]));
				tmPnnC.push_back(boost::shared_array<double>(new double[thetas.size()]));
			}
			for (size_t i=0;i<4;i++)
			{
				mieSr.push_back(boost::shared_array<double>(new double[thetas.size()]));
				mieSi.push_back(boost::shared_array<double>(new double[thetas.size()]));
				tmSr.push_back(boost::shared_array<double>(new double[thetas.size()]));
				tmSi.push_back(boost::shared_array<double>(new double[thetas.size()]));
				tmSrC.push_back(boost::shared_array<double>(new double[thetas.size()]));
				tmSiC.push_back(boost::shared_array<double>(new double[thetas.size()]));
			}

			size_t i=0;
			for (auto theta = thetas.begin(); theta != thetas.end(); ++theta, ++i)
			{
				double thetar = *theta * boost::math::constants::pi<double>() / 180.0;
				double mu = cos(thetar);

				double mSnn[4][4], mPnn[4][4];
				complex<double> mSn[4];
				mS.calc(mu,mSnn,mSn); // unnormalized pf values
				auto mmPnn = mP.eval(thetar); // mie normalized pf
				mmPnn->toDoubleArray(&mPnn[0][0]);
				
				tm.vars.THET = *theta;
				if (doTm)
					tm.run();

				// Stupid, but it really helps ROOT plotting

				for (size_t k=0;k<4;k++)
				{
					mieSr[k][i] = mSn[k].real();
					mieSi[k][i] = mSn[k].imag();
					tmSr[k][i] = tm.outs.S[k].real();
					tmSi[k][i] = tm.outs.S[k].imag();

					tmSiC[k][i] = tm.outs.S[k].imag();
					tmSrC[k][i] = tm.outs.S[k].real();

					accsSr[k](mieSr[k][i]);
					accsSr[k](tmSr[k][i]);
					accsSr[k](mieSi[k][i]);
					accsSr[k](tmSi[k][i]);
				}

				// Do my own pf calculation
				if (doTm && 0)
				{
					complex<double> Sn[4];
					double Snn[4][4];
					for (size_t k=0; k<4; k++)
						Sn[k] = complex<double>(tmSrC[k][i],tmSiC[k][i]);
					rtmath::scattMatrix::_genMuellerMatrix(Snn,Sn);
					// And divide by the normalizing factor
					mie::Qcalc qc(M);
					double Qext, Qsca, Qabs, g;
					qc.calc(*X,Qext, Qsca, Qabs, g);
					for (size_t k=0;k<4;k++)
						for (size_t l=0;l<4;l++)
						{
							tmPnnC[(4*k)+l][i] = 
								4.0 * Snn[k][l] / (*X * *X * Qext);
						}
				}

				// Use mishchenko's pf calculation

				// And the default calculated stuff
				for (size_t k=0;k<16;k++)
				{
					mieSnn[k][i] = mSnn[k/4][k%4];
					miePnn[k][i] = mPnn[k/4][k%4];
					tmPnn[k][i] = tm.outs.P[k/4][k%4];
					accsP[k](miePnn[k][i]);
					accsP[k](tmPnn[k][i]);
				}

			}
			
			boost::shared_ptr<TCanvas> tS(new TCanvas("S","S", 800*4,800*2));
			gStyle->SetPalette(1);
			tS->Divide(4,2);

			//size_t color = 1;

			vector<boost::shared_ptr<TGraph> > graphs;
			graphs.reserve(48);
			vector<boost::shared_ptr<TLegend> > legends;
			legends.reserve(24);

			for (Int_t j=0; j<4; j++)
			{
				ostringstream rTitle, iTitle;
				Int_t iA = (j / 2) + 1;
				Int_t iB = (j % 2) + 1;
				rTitle << "Real S_{" << iA << iB << "}";
				iTitle << "Imaginary S_{" << iA << iB << "}";
				boost::shared_ptr<TGraph> gMr(new TGraph(thetas.size(), Othetas.get(), mieSr[j].get()));
				boost::shared_ptr<TGraph> gMi(new TGraph(thetas.size(), Othetas.get(), mieSi[j].get()));
				boost::shared_ptr<TGraph> gTr(new TGraph(thetas.size(), Othetas.get(), tmSr[j].get()));
				boost::shared_ptr<TGraph> gTi(new TGraph(thetas.size(), Othetas.get(), tmSi[j].get()));
//				boost::shared_ptr<TGraph> gTrC(new TGraph(thetas.size(), Othetas.get(), tmSrC[j].get()));
//				boost::shared_ptr<TGraph> gTiC(new TGraph(thetas.size(), Othetas.get(), tmSiC[j].get()));


				graphs.push_back(gMr);
				graphs.push_back(gMi);
				graphs.push_back(gTr);
				graphs.push_back(gTi);
//				graphs.push_back(gTrC);
//				graphs.push_back(gTiC);
				TPad *pad = tS->cd(j+1);
				pad->SetFillStyle(4000);

				gMr->Draw("AC");
				gMr->SetLineColor(1);
				gMr->SetTitle(rTitle.str().c_str());
				gMr->GetXaxis()->SetTitle("#Theta");
				gMr->GetXaxis()->CenterTitle();
				gMr->GetYaxis()->SetTitle("S_{real}");
				gMr->GetYaxis()->CenterTitle();
				double MIN = boost::accumulators::min(accsSr[j]);
				double MAX = boost::accumulators::max(accsSr[j]);
//				cerr << MIN << "\t" << MAX << endl;
				if (abs(MAX-MIN) < 0.01) MAX += 0.05;
				gMr->GetHistogram()->SetMinimum(MIN);
				gMr->GetHistogram()->SetMaximum(MAX);
//				gMr->GetYaxis()->SetLimits(MIN,MAX);
				gMr->Draw("AC");
				if (doTm)
				{
					gTr->Draw("C");
//					gTrC->Draw("C");
				}
				gTr->SetLineColor(2);

				boost::shared_ptr<TLegend> lSr(new TLegend(0.7,0.8,0.9,0.9));
				legends.push_back(lSr);
				lSr->AddEntry(gMr.get(),"Mie Theory", "l");
				if (doTm)
					lSr->AddEntry(gTr.get(),"T-Matrix", "l");
				lSr->Draw();


//				gTrC->SetLineColor(3);
				pad = tS->cd(5+j);
				pad->SetFillStyle(4000);
				gMi->Draw("AC");
				gMi->SetLineColor(1);
				gMi->SetTitle(iTitle.str().c_str());
				gMi->GetXaxis()->SetTitle("#Theta");
				gMi->GetXaxis()->CenterTitle();
				gMi->GetYaxis()->SetTitle("S_{imag}");
				gMi->GetYaxis()->CenterTitle();
//				gMi->GetYaxis()->SetLimits(MIN,MAX);
				gMi->GetHistogram()->SetMinimum(MIN);
				gMi->GetHistogram()->SetMaximum(MAX);

				gMi->Draw("AC");

				if (doTm)
				{
					gTi->Draw("C");
//					gTiC->Draw("C");
				}
				gTi->SetLineColor(2);
				boost::shared_ptr<TLegend> lSi(new TLegend(0.7,0.8,0.9,0.9));
				legends.push_back(lSi);
				lSi->AddEntry(gMi.get(),"Mie Theory", "l");
				if (doTm)
					lSi->AddEntry(gTi.get(),"T-Matrix", "l");
				lSi->Draw();


//				gTiC->SetLineColor(3);
			}
			TPad *pad = tS->cd();


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
//					boost::shared_ptr<TGraph> tPC(new TGraph(thetas.size(), Othetas.get(), tmPnnC[(4*j)+k].get()));

					graphs.push_back(gP);
					graphs.push_back(tP);
//					graphs.push_back(tPC);
					tcP->cd((4*j)+k+1);
					gP->Draw("AC");
					gP->SetLineColor(1);
					gP->SetTitle(title.str().c_str());
					gP->GetXaxis()->SetTitle("#Theta");
					gP->GetXaxis()->CenterTitle();
					gP->GetYaxis()->SetTitle(title.str().c_str());
					gP->GetYaxis()->CenterTitle();
					double MIN = boost::accumulators::min(accsP[(4*j)+k]);
					double MAX = boost::accumulators::max(accsP[(4*j)+k]);
					if (abs(MAX-MIN) < 0.01) MAX += 0.05;
//					gP->GetYaxis()->SetLimits(MIN,MAX);
					gP->GetHistogram()->SetMinimum(MIN);
					gP->GetHistogram()->SetMaximum(MAX);

					gP->Draw("AC");

					if (doTm)
					{
						tP->Draw("C");
//						tPC->Draw("C");
					}
					tP->SetLineColor(2);
//					tPC->SetLineColor(3);
					boost::shared_ptr<TLegend> lP(new TLegend(0.7,0.6,0.9,0.7));
					legends.push_back(lP);
					lP->AddEntry(gP.get(),"Mie Theory", "l");
					if (doTm)
						lP->AddEntry(tP.get(),"T-Matrix", "l");
					lP->Draw();


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

