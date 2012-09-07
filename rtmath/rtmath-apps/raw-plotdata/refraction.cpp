#include <string>
#include <iostream>
#include <memory>
#include <set>
#include <map>
#include <complex>
#include <cmath>
#define BOOST_TEST_DYN_LINK

#include "../../rtmath/rtmath/ROOTlink.h"

#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/splitSet.h"

#include <boost/test/unit_test.hpp>
#include <boost/shared_array.hpp>
#include <boost/shared_ptr.hpp>

BOOST_AUTO_TEST_SUITE(refraction);

using namespace std;
using namespace rtmath;
using namespace rtmath::debug;

BOOST_AUTO_TEST_CASE(sihvola)
{
	const double freq = 94; // GHz
	const double T = 263;
	complex<double> Mair(1.0,0);
	complex<double> Mice;
	rtmath::refract::mice(freq,T,Mice);

	set<double> sf;
	rtmath::config::splitSet("0:50:1:lin", sf);
	set<double> snu;
	rtmath::config::splitSet("0:0.4:2.0", snu);

	boost::shared_ptr<TCanvas> tC(new TCanvas("c","PE plot", 0,0,2000,1400));
	gStyle->SetPalette(1);

	boost::shared_array<double> f(new double[sf.size()]);
	vector<boost::shared_array<double> > d;
	size_t i=0;
	size_t k=1;
	for (auto it = sf.begin(); it != sf.end(); ++it, ++i)
		f[i] = *it;
	
	for (auto it = snu.begin(); it != snu.end(); ++it, ++k)
	{
		// it represents the value of nu
		double *data = new double[sf.size()];
		i=0;
		//boost::shared_array<double> data(new double[sf.size()]);

		for (auto ot = sf.begin(); ot != sf.end(); ++ot, ++i)
		{
			complex<double> Mres;
			double frac = *ot;

			rtmath::refract::sihvola(Mice,Mair,frac,*it,Mres);
			
			data[i] = Mres.real();
			//if (data[i] > 1.0 || data[i] < 0) data[i] = 0;
		}
		//d.push_back(data);
		TGraph* tM = (new TGraph(sf.size(),f.get(),data));
		if (it == snu.begin())
		{
			tM->Draw("AL");
			tM->SetTitle("Mass vs. Effective Radius");
			tM->GetXaxis()->SetTitle("Volume fraction");
			tM->GetXaxis()->CenterTitle();
			tM->GetYaxis()->SetTitle("Reff_real");
			tM->GetYaxis()->CenterTitle();
		} else {
			tM->Draw("L");
		}
		tM->SetLineColor(k);
		//tC->SaveAs("sihvola.png");
	}

	

	tC->SaveAs("sihvola.png");
	
}

BOOST_AUTO_TEST_CASE(e_f_data)
{
	/*
	const double T = 260; // K
	const double freq = 35.6; // GHz
	complex<double> Matm(1,0);
	complex<double> Mice;
	rtmath::refract::mice(freq,T,Mice);

	complex<double> e_atm = Matm * Matm;
	complex<double> e_ice = Mice * Mice;

	ofstream out("e-260K-35.6GHz.dat");
	out << "fraction\te_atm_r\te_atm_i\te_ice_r\te_ice_imag\tMice_real\tMice_imag\te_r\te_i\tM-re\tM_im" << endl;
	for (double f=0; f<=1.0; f += 0.01)
	{
		complex<double> ea = f * (e_ice - complex<double>(1.,0)) / (e_ice + complex<double>(2.,0));
		complex<double> eb = (1.-f) * (e_atm - complex<double>(1.,0)) / (e_atm + complex<double>(2.,0));

		complex<double> fact = ea+eb;
		complex<double> e = (complex<double>(2.,0) * fact + complex<double>(1.,0)) 
			/ (complex<double>(1.,0) - fact);

		out << f << "\t" << e_atm.real() << "\t" << e_atm.imag() << "\t" << e_ice.real() << "\t" << 
			e_ice.imag() << "\t" << Mice.real() << "\t" << Mice.imag() << "\t" << 
			e.real() << "\t" << e.imag() << "\t" << 
			sqrt(e).real() << "\t" << sqrt(e).imag() << endl;
		
	}*/
}




BOOST_AUTO_TEST_SUITE_END();

