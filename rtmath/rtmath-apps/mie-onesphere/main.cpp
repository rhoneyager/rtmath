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
			("aeffs,a", po::value<std::string>(), "Specify the effective radii in um")
			("temps,T", po::value<std::string>()->default_value("263"), "Specify temperatures in K")
			("freqs,f", po::value<std::string>(), "Specify frequencies in GHz. Needed for dielectrics.")
			("output-prefix,o", po::value<string>()->default_value("output"), "Set output file (required)")
			("volume-fractions,v", po::value<std::string>()->default_value("1"), "Set the ice volume fractions")
			("nu,n", po::value<double>()->default_value(0.85), "Value of nu for Sihvola refractive index scaling")
			("scale-aeff", "Scale effective radius based on volume fraction")
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

		//double vfrac = vm["volume-fractions"].as<double>();
		bool scale = false;
		if (vm.count("scale-aeff")) scale = true;

		double nu = vm["nu"].as<double>();

		set<double> temps, freqs, aeffs, vfracs;
		rtmath::config::splitSet(vm["temps"].as<string>(), temps);
		rtmath::config::splitSet(vm["freqs"].as<string>(), freqs);
		rtmath::config::splitSet(vm["aeffs"].as<string>(), aeffs);
		rtmath::config::splitSet(vm["volume-fractions"].as<string>(), vfracs);


		// The actual runs begin here
		using namespace rtmath::mie;
		using namespace std;

		ofstream out( string(oprefix).append(".csv").c_str());
		out << "Temperature (K),Frequency (GHz),Effective Radius (um),"
			"mrr,mri,Volume Fraction,nu,Size Parameter,Qabs,Qsca,Qext,g,Qbk" << endl;

		for (const double &temp : temps)
		for (const double &freq : freqs)
		for (const double &aeff : aeffs)
		for (const double &vfrac : vfracs)
		{
			ostringstream ofiless;
			// the prefix is expected to be the discriminant for the rotational data
			ofiless << oprefix << "-t-" << temp << "-f-" << freq 
				<< "-aeff-" << aeff << "-nu-" << nu
				<< "-vfrac-" << vfrac;
			string ofile = ofiless.str();
			cout << ofile << endl;

			complex<double> mIce, m, mAir(1,0);
			rtmath::refract::mIce(freq, temp, mIce);

			rtmath::refract::sihvola(mIce,mAir,vfrac,nu,m);

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

			mieBase base;
			base.AXI = saeff;
			base.LAM = lam;
			base.MRR = m.real();
			base.MRI = abs(m.imag());
			boost::shared_ptr<const mieParams> params = mieParams::create(base);

			// No need for orientations, as spheres are rotation invariant

			auto res = mieCalc::calc(params);

			Qabs += res->qabs;
			Qsca += res->qsca;
			Qext += res->qext;
			Qbk += res->qbk;
			g += res->g;

			// Just for compatibility
			//mieAngleRes::calc(res, 180);
			//auto angres = mieAngleRes::calc(res, 180);
			//cout << "P11 = " << angres->getP(0,0) << endl;
			//cout << "Qbksc,unpol = " << getDifferentialBackscatterCrossSectionUnpol(res) << endl;
			
			out << temp << "," << freq << "," << aeff << ","
				<< m.real() << "," << abs(m.imag()) << "," << vfrac << "," << nu << "," << sizep << ","
				<< Qabs << "," << Qsca << "," << Qext << "," << g << "," << Qbk << endl;
			
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



