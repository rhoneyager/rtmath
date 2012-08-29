/* atmosplot
 * Program designed for making plots about atmospheric profile quantities,
 * for selected profiles while varying T, p, RH for many gases.
 */

#include <cmath>
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include "../../rtmath/rtmath/ROOTlink.h"
// root headers are identical to those in ROOTlink.h
#include "../../rtmath/rtmath/absorb.h"
#include "../../rtmath/rtmath/atmos.h"
#include "../../rtmath/rtmath/command.h"
#include "../../rtmath/rtmath/config.h"

void doHelp();

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;

	try 
	{
		cerr << "rtmath-atmosplot\n\n";
		rtmath::debug::appEntry(argc, argv);
		if (argc == 1) doHelp();
		config::parseParams p(argc,argv);

		if (p.readParam("-h")) doHelp();

		bool flag = false;

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
		
		// Get master configuration file path
		// Used to find out where HITRAN is and where the profile paths are
		string rtconfpath;
		flag = p.readParam<string>("--config", rtconfpath);
		cerr << "Loading config file." << endl;
		// Read in configuration file
		std::shared_ptr<config::configsegment> cRoot = config::loadRtconfRoot(rtconfpath);
		string profdir, profext;
		// Get profile directory
		cRoot->getVal("atmos/atmosDir", profdir);
		if (profdir == "") profdir == "./";
		// If profile search directory is user-specified, override
		p.readParam<string>("--profdir", profdir); 
		cerr << "Profile search directories: " << profdir << endl;
		// Get profile extensions
		cRoot->getVal("atmos/profileExtensions",profext);

		// Get profile name for calculations
		string profname;
		flag = p.readParam<string>("--profile", profname);
		if (!flag) doHelp();
		cerr << "Profile desired: " << profname << ". Searching..." << endl;
		// Find the actual path to the atmospheric profile
		string profpath;
		{
			rtmath::config::findFile psearch(profdir, profext);
			bool found = psearch.search(profname,profpath);
			if (!found) throw debug::xMissingFile(profname.c_str());
		}

		cerr << "Profile found in " << profpath << endl;

		

		// Read atmospheric profile
		atmos::atmos atm;
		atm.loadProfile(profpath.c_str());

		// Get frequency map
		unique_ptr<double[]> f(new double[freqs.size()]);
		size_t i = 0;
		for (auto it = freqs.begin(); it != freqs.end(); ++it, ++i)
			f[i] = *it;

		{
			// Basic profile plots:

			// Get pressure set
			// Get temperature set
			// Get rh set
			size_t np = atm._layers.size();
			unique_ptr<double[]> p(new double[np]);
			unique_ptr<double[]> T(new double[np]);
			unique_ptr<double[]> rh(new double[np]);
			unique_ptr<double[]> z(new double[np]);
			for (i=0;i<atm._layers.size();i++)
			{
				p[i] = atm._layers[i].p();
				T[i] = atm._layers[i].T();
				z[i] = atm._layers[i].z();
				//rh[i] = atm._layers[i].
				double RH = rtmath::atmos::absorber::_RH
					(T[i],atm._layers[i].wvden());
				rh[i] = RH;
			}
			// Get gas conc set




			// Begin plotting

			TCanvas Ctp("Ctpar","Temp vs (pressure, alt, rh)", 3000, 1600);
			Ctp.Divide(3,1);
			// Temp. vs p
			//TVirtualPad *gPad;
			Ctp.cd(1);
			TGraph gTP((Int_t) np,T.get(),p.get());
			gTP.GetXaxis()->SetTitle("Temperature (K)");
			gTP.GetXaxis()->CenterTitle();
			gTP.GetYaxis()->SetTitle("Pressure (hPa)");
			gTP.GetYaxis()->CenterTitle();
			gPad->SetLogy(1);
			ostringstream gTPtitle;
			gTPtitle << atm.name;
			gTP.SetTitle(gTPtitle.str().c_str());
			gTP.Draw("AL");
			//ROOT_funcs::flip_axis(&gTP);

			// Alt. vs p
			Ctp.cd(2);
			TGraph gAP((Int_t) np,z.get(),p.get());
			gAP.GetXaxis()->SetTitle("Altitude (km)");
			gAP.GetXaxis()->CenterTitle();
			gAP.GetYaxis()->SetTitle("Pressure (hPa)");
			gAP.GetYaxis()->CenterTitle();
			gPad->SetLogy(1);
			ostringstream gAPtitle;
			gAPtitle << atm.name;
			gAP.SetTitle(gTPtitle.str().c_str());
			gAP.Draw("AL");
			//ROOT_funcs::flip_axis(&gAP);

			// RH vs p
			Ctp.cd(3);
			TGraph gRP((Int_t) np,rh.get(),p.get());
			gRP.GetXaxis()->SetTitle("Relative Humidity (%)");
			gRP.GetXaxis()->CenterTitle();
			gRP.GetYaxis()->SetTitle("Pressure (hPa)");
			gRP.GetYaxis()->CenterTitle();
			gPad->SetLogy(1);
			ostringstream gRPtitle;
			gRPtitle << atm.name;
			gRP.SetTitle(gTPtitle.str().c_str());
			gRP.Draw("AL");
			//ROOT_funcs::flip_axis(&gRP);

			ostringstream fbasic;
			fbasic << profname << "-basic.png";
			Ctp.SaveAs(fbasic.str().c_str());

			// gas concentrations vs p
		}

		{
			// Frequency-varied plots

		}

		if (p.readParam("-pi")) {
			// Profile-independent (gas-only) plots
			const Int_t n = 6;
			const double tfixed[] = { 243, 253, 263, 273, 293, 303 };
			const double pfixed[] = { 1000, 850, 700, 500, 300, 200 };
			const double dzfixed = 1.0; // only care about a constant thickness
			const Int_t nf = (Int_t) freqs.size();
			unique_ptr<double[]> f(new double[freqs.size()]);
			for (auto it = freqs.begin(); it != freqs.end(); ++it, ++i)
				f[i] = *it;
			set<string> gases;
			{
				vector<string> sGases;
				p.readParam<string>("-g", sGases);
				if (sGases.size() == 0)
					sGases.push_back("H2O,O2,N2,COLLIDE"); // Default gases
				for (auto it = sGases.begin(); it != sGases.end(); ++it)
					rtmath::config::splitSet<string>(*it,gases);
			}

			// Do the analyses for each gas
			for (auto it = gases.begin(); it != gases.end(); it++)
			{
				// Construct a one-layer atmosphere

				// freq vs p vs tau
				
				// freq vs t vs tau

				// p vs t vs tau (selected frequencies)
			}
		}

		if (p.readParam("-pi")) 
		{
			// Profile and gas indep plots
			// Relative humidity calculation and inverse
			// Relative humidity set
			TCanvas Crh("Crh","WV density", 3000, 1600);
			TGraph2D dt;
			double rh, t, wvden;
			int n = 0;
			
			dt.SetNpx(21);
			dt.SetNpy(31);
			for (rh =0;rh<=100;rh+=5)
			{
				for (t=-50;t<=100;t+=5)
				{
					double tk = t + 273.15;
					wvden = atmos::absorber::_Vden(tk,rh);
					dt.SetPoint(n,rh,t,wvden);
					n++;
				}
			}
			dt.GetXaxis()->SetTitle("Relative Humidity (%)");
			dt.GetYaxis()->SetTitle("Temperature (C)");
			//dt.GetZaxis()->SetTitle("Water vapor concentration (g/m^3)");
			gStyle->SetPalette(1);
			dt.SetTitle("Water vapor concentration (g/m^3)");
			dt.Draw("CONT4Z");
			Crh.SaveAs("rh-t-wvden.png");

			// Inverse plot
			// Take water vapor conc and convert to RH
			TCanvas Crhi("Crhi","WV density", 3000, 1600);
			TH2D dti("dti","dti",71,0,700,21,0,100);
			dti.SetBit(TH1::kCanRebin);
			for (wvden =0;wvden<=700;wvden+=10)
			{
				Int_t a = (Int_t) wvden / 10;
				for (t=0;t<=100;t+=5)
				{
					Int_t b = ((Int_t) t) / 5;
					double tk = t + 273.15;
					rh = atmos::absorber::_RH(tk,wvden);
					if (rh > 100) rh = 100;
					dti.SetBinContent(a, b, rh);
					//dti.AddBinContent(wvden,t,rh);
					n++;
				}
			}
			dti.GetXaxis()->SetTitle("Water vapor concentration (g/m^3)");
			dti.GetYaxis()->SetTitle("Temperature (C)");
			gStyle->SetPalette(1);
			dti.SetTitle("Relative Humidity (%)");
			dti.SetStats(false);
			dti.Draw("CONT4Z");
			Crhi.SaveAs("rh-t-wvden-inv.png");
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
	cout << "Program designed for making plots about atmospheric profile,\n";
	cout << "quantities for selected profiles while varying T, p, RH for \n";
	cout << "many gases.\n";
	cout << "Options:\n";
	cout << "-f (frequency range)\n";
	cout << "\tSpecify the range of frequencies (in GHz) for calculation\n";
	cout << "--profile " << endl;
	cout << "\tSpecify the desired atmospheric profile. Consult\n";
	cout << "\tthe manual for the file format.\n";
	cout << "--profdir\n";
	cout << "\tOverride the search directory for atmospheric\n";
	cout << "\tprofiles. By default, we look in config and ./\n";
	cout << "-pi\n";
	cout << "\tAlso generate basic profile-independent plots.\n";
	cout << "-h\n";
	cout << "\tProduce this help message.\n";
	cout << endl << endl;
	exit(1);
}

