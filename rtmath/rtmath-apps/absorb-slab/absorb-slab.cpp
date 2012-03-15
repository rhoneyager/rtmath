/* absorb-slab
* A program designed to go through the absorption functions and test their domains for a sample, 
* really crude bulk atmosphere. The block atmosphere is a 5 km-thick slab with 700 mb pressure.
* It is at 273 K. Rel. humid. is at 15%. It will show if any of the referenced absorption 
* functions is misbehaving. */

#include <cmath>
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include "../../rtmath/rtmath/rtmath.h"
#include <TGraph.h>
#include <TF1.h>
#include <TCanvas.h>
#include <TAxis.h>
#include <TNamed.h>

void doHelp();

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace rtmath::atmos;

	try {
		cerr << "rtmath-absorb-slab\n\n";
		rtmath::debug::appEntry(argc, argv);
		if (argc == 1) doHelp();
		config::parseParams p(argc,argv);

		bool flag = false;
		bool doPlots = false;

		p.readParam("-h",flag);
		if (flag) doHelp();

		double rh = 0, T = 273, pres = 700, dz = 5;
		set<double> freqs;
		{
			vector<string> vF;
			flag = p.readParam<string>("-f", vF);
			if (!flag)
			{
				cerr << "Frequency range must be specified.\n";
				doHelp();
			}
			for (auto it = vF.begin(); it != vF.end(); it++)
				rtmath::config::splitSet<double>(*it,freqs);
		}

		set<string> gases;
		{
			vector<string> sGases;
			flag = p.readParam<string>("-g", sGases);
			if (sGases.size() == 0)
				sGases.push_back("H2O,O2,N2,COLLIDE"); // Default gases
			for (auto it = sGases.begin(); it != sGases.end(); ++it)
				rtmath::config::splitSet<string>(*it,gases);
		}

		if (gases.count("H2O")) rh = 15; // Set new default value
		p.readParam<double>("-rh", rh);
		p.readParam<double>("-t", T);
		p.readParam<double>("-p", pres);
		p.readParam<double>("-dz", dz);

		// Should plots over the requested frequency domain be done?
		string plotfile;
		doPlots = p.readParam<string>("-P",plotfile);

		// Build the sample atmosphere
		rtmath::atmos::atmos a;

		a._layers.resize(1);

		atmoslayer *layer = &a._layers[0];
		layer->dz(dz);
		layer->p(pres);
		layer->T(T);

		// Insert the gases into the layer
		{
			// Convert relative humidity into rho_Wat
			double rhoWat = 0;
			rhoWat = absorber::_Vden(layer->T(), rh);
			for (auto it = gases.begin(); it != gases.end(); ++it)
			{
				std::shared_ptr<absorber> newgas;
				// Find the most appropriate absorber
				absorber::_findAbsorber(*it,newgas);
				newgas->setLayer(*layer);
				newgas->wvden(rhoWat);
				layer->absorbers.insert(newgas);
			}
		}

		// Okay, now run the atmosphere for the frequency range requested and report results.
		{
			cerr << "Running atmosphere\n";
			cerr << "frequency (GHz), tau (nepers)" << endl;

			// Loop through freqs
			set<double>::const_iterator it;
			map<double,double> taus;
			for (it = freqs.begin(); it != freqs.end(); it++)
			{
				double tau = a.tau(*it);
				taus[*it] = tau;
				cout << *it << "," << tau << endl;
			}

			if (doPlots)
			{
				// Use ROOT to plot a graph of the data
				// Unfortunately, ROOT has no support for std::pair or std::map.....
				// Must convert to doubles.
				unique_ptr<double[]> f(new double[taus.size()]);
				unique_ptr<double[]> t(new double[taus.size()]);
				unique_ptr<double[]> Tr(new double[taus.size()]);
				size_t i = 0;
				for (auto it = taus.begin(); it != taus.end(); ++it, ++i)
				{
					f[i] = it->first;
					t[i] = it->second;
					Tr[i] = exp(-1.0 * it->second);
				}

				TCanvas c1("c1","canvas", 3000, 1600);
				//c1.Size(60,80);
				c1.Divide(1,2);
				c1.cd(1);
				TGraph gTaus((Int_t) taus.size(),f.get(),t.get());
				gTaus.GetXaxis()->SetTitle("Frequency (GHz)");
				gTaus.GetXaxis()->CenterTitle();
				gTaus.GetYaxis()->SetTitle("#tau (nepers)");
				gTaus.GetYaxis()->CenterTitle();
				// Set plot title
				ostringstream sTitle;
				for (auto ot = gases.begin(); ot != gases.end(); ot++)
				{
					if (ot != gases.begin()) sTitle << ", ";
					sTitle << *ot;
				}
				sTitle << " at " << pres << " hPa, " << T << " C, " 
					<< rh << "% rh, " << dz << " km thickness";
				gTaus.SetTitle(sTitle.str().c_str());
				gTaus.Draw("AL");

				c1.cd(2);
				TGraph gTr((Int_t) taus.size(),f.get(),Tr.get());
				gTr.GetXaxis()->SetTitle("Frequency (GHz)");
				gTr.GetXaxis()->CenterTitle();
				gTr.GetYaxis()->SetTitle("Transmittance");
				gTr.GetYaxis()->CenterTitle();
				gTr.Draw("AL");
				gTr.SetTitle(" ");


				c1.SaveAs(plotfile.c_str());
			}
		}
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		cerr << endl;
		return 1;
	}
	return 0;
}


void doHelp()
{
	using namespace std;
	cout << "A program for calculating transmittance in a sample atmosphere,\n";
	cout << "to determine the domains of the absorbtion-calculationg functions.\n";
	cout << "Options:\n";
	cout << "-f (frequency range)\n";
	cout << "\tSpecify the range of frequencies (in GHz) for\n";
	cout << "\ttransmittance calculation. " << endl;
	cout << "-g (gases)\n";
	cout << "\tManually specify the gases to place in the atmosphere.\n";
	cout << "\tAcceptable values: H2O, N2, O2, COLLIDE\n";
	cout << "-rh (relative humidity as a percent)\n";
	cout << "\tSpecify relative humidity from 0 to 100.\n";
	cout << "\tWorks even without H2O, as some gas calculations \n";
	cout << "\t(like O2) depend on this value.\n";
	cout << "\tDefault is 0 w/o water, 15 w/water.\n";
	cout << "-t (temp, K)\n";
	cout << "\tTemperature (default = 273 K)\n";
	cout << "-p (pressure, hPa)\n";
	cout << "\tPressure (default = 700 hPa)\n";
	cout << "-dz (depth, km)\n";
	cout << "\tSlab thickness (defult 5 km)\n";
	cout << "-P (filename)\n";
	cout << "\tProduce plots over the requested frequency domain.\n";
	cout << "-h\n";
	cout << "\tProduce this help message.\n";
	cout << endl << endl;
	exit(1);
}

