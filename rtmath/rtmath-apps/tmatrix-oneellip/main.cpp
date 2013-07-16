#include <iostream>
#include <boost/program_options.hpp>
#include <boost/math/constants/constants.hpp>
#include <fstream>
#include <complex>
#include <set>
#include <vector>
#include <sstream>
#include <string>
#include <Ryan_Debug/debug.h>
#include <tmatrix/tmatrix.h>
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/mie/mie.h"
#include "../../rtmath/rtmath/mie/mie-Qcalc.h"
#include "../../rtmath/rtmath/units.h"

int main(int argc, char *argv[])
{
	using namespace std;
	try {
		cerr << argv[0]<< endl;
		// Do processing of argv
		namespace po = boost::program_options;
		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("aspect-ratios,s", po::value<std::string>()->default_value("1"), "Specify aspect ratio for ellipsoids.")
			("aeffs,a", po::value<std::string>(), "Specify the effective radii in um")
			("temps,T", po::value<std::string>()->default_value("263"), "Specify temperatures in K")
			("freqs,f", po::value<std::string>(), "Specify frequencies in GHz. Needed for dielectrics.")
			("output-prefix,o", po::value<string>()->default_value("output"), "Set output file (required)")
			("volume-fractions,v", po::value<std::string>()->default_value("1"), "Set the ice volume fractions")
			("nu,n", po::value<double>()->default_value(0.85), "Value of nu for Sihvola refractive index scaling")
			("scale-aeff", "Scale effective radius based on volume fraction")
			("mr", po::value<double>(), "Override real refractive index value")
			("mi", po::value<double>(), "Override imaginary refractive index value")
			("alphas", po::value<string>()->default_value("0"), "Set beta rotations for backscatter calculation. UNDER DEVELOPMENT.")
			("betas", po::value<string>()->default_value("0"), "Set second rotation for backscatter calculation. UNDER DEVELOPMENT.")
		;

		po::variables_map vm;
		po::store(po::command_line_parser(argc,argv).options(desc).run(), vm);
		po::notify(vm);

		// Begin checking parameters
		auto doHelp = [&](const std::string& s)
		{
			cout << s << endl;
			cout << desc << endl;
			exit(3);
		};

		if (vm.count("help") || vm.size() == 0) doHelp("");

		string oprefix;
		if (vm.count("output-prefix"))
			oprefix = vm["output-prefix"].as<string>();

		if (!vm.count("freqs")) doHelp("Need to specify frequencies.");
		if (!vm.count("aeffs")) doHelp("Need to specify effective radii.");

		//double vfrac = vm["volume-fraction"].as<double>();
		bool scale = false;
		if (vm.count("scale-aeff")) scale = true;

		double nu = vm["nu"].as<double>();

		bool overrideM = false;
		complex<double> ovM;
		if (vm.count("mr") || vm.count("mi"))
		{
			overrideM = true;
			if (!vm.count("mr") || !vm.count("mi"))
				doHelp("When overriding refractive index, need to specify "
					"both real and imaginary components.");
			double r = vm["mr"].as<double>();
			double i = vm["mi"].as<double>();
			ovM = complex<double>(r,i);
			nu = -1.0;
		}

		set<double> temps, freqs, aeffs, aspects, vfracs, alphas, betas;
		rtmath::config::splitSet(vm["temps"].as<string>(), temps);
		rtmath::config::splitSet(vm["freqs"].as<string>(), freqs);
		rtmath::config::splitSet(vm["aeffs"].as<string>(), aeffs);
		rtmath::config::splitSet(vm["aspect-ratios"].as<string>(), aspects);
		rtmath::config::splitSet(vm["volume-fractions"].as<string>(), vfracs);
		rtmath::config::splitSet(vm["alphas"].as<string>(), alphas);
		rtmath::config::splitSet(vm["betas"].as<string>(), betas);

		// The actual runs begin here
		using namespace tmatrix;
		//using namespace rtmath::mie;
		using namespace std;

		ofstream out( string(oprefix).append(".csv").c_str());
		out << "Temperature (K),Frequency (GHz),Effective Radius (um),Aspect Ratio,"
			"mrr,-mri,Volume Fraction,nu,Size Parameter,Qabs,Qsca,Qext,Qbk" << endl;

		for (const double &temp : temps)
		for (const double &freq : freqs)
		for (const double &aeff : aeffs)
		for (const double &vfrac : vfracs)
		for (const double &aspect : aspects)
		{
			ostringstream ofiless;
			// the prefix is expected to be the discriminant for the rotational data
			ofiless << oprefix << "-t-" << temp << "-f-" << freq 
				<< "-aeff-" << aeff << "-nu-" << nu 
				<< "-vfrac-" << vfrac << "-aspect-" << aspect;
			string ofile = ofiless.str();
			cout << ofile << endl;

			complex<double> mIce, m, mAir(1,0);
			rtmath::refract::mIce(freq, temp, mIce);
			if (overrideM)
			{
				m = ovM;
			} else {
				rtmath::refract::sihvola(mIce,mAir,vfrac,nu,m);
			}

			double saeff = aeff;
			if (scale)
			{
				double sV = pow(aeff,3.0);
				sV /= vfrac;
				saeff = pow(sV,1./3.);
			}

			double lam = rtmath::units::conv_spec("GHz","um").convert(freq);
			const double sizep = 2. * boost::math::constants::pi<double>() * aeff / lam;
			
			double Qbk = 0; // Will be divided over the number of rotations later
			double Qext = 0;
			double Qsca = 0;
			double Qabs = 0;
			double g = 0;

			tmatrixBase base;
			base.AXI = saeff;
			base.LAM = lam;
			base.MRR = m.real();
			base.MRI = abs(m.imag());
			base.EPS = aspect;
			if ( abs(aspect - 1.0) < 0.00001 ) base.EPS = 1.0001;
			boost::shared_ptr<const tmatrixParams> params = tmatrixParams::create(base);

			const double nRots = (double) alphas.size() * (double) betas.size();

			for (const double &alpha : alphas)
			for (const double &beta : betas)
			{
				auto res = OriTmatrix::calc(params, alpha, beta);
				Qsca += res->qsca / nRots;
				Qext += res->qext / nRots;
				Qbk += getDifferentialBackscatterCrossSectionUnpol(res) / nRots;
			}

			Qabs = Qext - Qsca;
			g = 0;


			// FIX: scaling relation
			//double k = sizep / aeff; // = 2pi / wvlen
			//Qbk *= k * k;
			//Qbk /= aeff;
			//Qbk /= boost::math::constants::pi<double>() * aeff * aeff;
			//Qbk *= sizep * sizep;

			// Just for compatibility
			//mieAngleRes::calc(res, 180);
			//auto angres = mieAngleRes::calc(res, 180);
			//cout << "P11 = " << angres->getP(0,0) << endl;
			//cout << "Qbksc,unpol = " << getDifferentialBackscatterCrossSectionUnpol(res) << endl;
			
			out << temp << "," << freq << "," << aeff << "," << aspect << ","
				<< m.real() << "," << abs(m.imag()) << "," << vfrac << "," 
				<< nu << "," << sizep << ","
				<< Qabs << "," << Qsca << "," << Qext << "," << Qbk << endl;
			
		}

		return 0;
	} 
	catch (std::exception &e)
	{
		cerr << "Exception: " << e.what() << endl;
		exit(2);
	}
	catch (...)
	{
		cerr << "Caught unidentified error... Terminating." << endl;
		exit(1);
	}
}



