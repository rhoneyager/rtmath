#include <iostream>
#include <boost/program_options.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/math/constants/constants.hpp>

#include <complex>
#include <set>
#include <vector>
#include <sstream>
#include <string>
#include <Ryan-Debug/debug.h>
#include <Ryan_Serialization/serialization.h>
#include <tmatrix/tmatrix.h>
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/ddscat/tmData.h"
#include "../../rtmath/rtmath/mie/mie.h"
#include "../../rtmath/rtmath/mie/mie-Qcalc.h"
#include "../../rtmath/rtmath/Serialization/tmData_serialization.h"
#include "../../rtmath/rtmath/units.h"

int main(int argc, char *argv[])
{
	using namespace std;
#ifdef _WIN32
	::serialization::disable_auto_compression(true);
#endif
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
			("volume-fraction,v", po::value<double>()->default_value(1), "Set the ice volume fraction")
			("no-xml", "Suppress xml output")
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

		bool noxml = false;
		if (vm.count("no-xml")) noxml = true;

		double vfrac = vm["volume-fraction"].as<double>();
		bool scale = false;
		if (vm.count("scale-aeff")) scale = true;

		set<double> betas, thetas, phis, temps, freqs, aspects, aeffs;
		rtmath::config::splitSet(vm["temps"].as<string>(), temps);
		rtmath::config::splitSet(vm["freqs"].as<string>(), freqs);
		rtmath::config::splitSet(vm["aeffs"].as<string>(), aeffs);

		// The actual runs begin here
		using namespace tmatrix;
		using namespace rtmath::mie;

		ofstream out( string(oprefix).append(".csv").c_str());
		out << "Temperature (K),Frequency (GHz),Effective Radius (um),"
			"mrr,mri,Volume Fraction,Size Parameter,Qbk,Qext,Qsca,g" << endl;

		for (const double &temp : temps)
		for (const double &freq : freqs)
		for (const double &aeff : aeffs)
		{
			ostringstream ofiless;
			// the prefix is expected to be the discriminant for the rotational data
			ofiless << oprefix << "-t-" << temp << "-f-" << freq 
				<< "-aeff" << aeff;
			string ofile = ofiless.str();
			cout << ofile << endl;

			complex<double> mIce, m, mAir(1,0);
			rtmath::refract::mIce(freq, temp, mIce);

			rtmath::refract::sihvola(mIce,mAir,vfrac,0.85,m);

			double saeff = aeff;
			if (scale)
			{
				double sV = pow(aeff,3.0);
				sV /= vfrac;
				saeff = pow(sV,1./3.);
			}

			rtmath::tmatrix::tmData data;
			data.T = temp;
			data.freq = freq;
			double lam = rtmath::units::conv_spec("GHz","um").convert(freq);
			const double sizep = 2. * boost::math::constants::pi<double>() * aeff / lam;
			data.sizep = sizep;
			data.reff = saeff;
			data.volMeth = "mie";
			data.dielMeth = "ice";
			data.shapeMeth = "ellipsoids";
			data.nu = 0.85;
			
			boost::shared_ptr<rtmath::tmatrix::tmStats> stats(new rtmath::tmatrix::tmStats);
			data.tstats = stats;

			double Qbk = 0; // Will be divided over the number of rotations later
			double Qext = 0;
			double Qsca = 0;
			double g = 0;

			mieBase base;
			base.AXI = saeff;
			base.LAM = lam;
			base.MRR = m.real();
			base.MRI = m.imag();
			boost::shared_ptr<const mieParams> params = mieParams::create(base);

			// No need for orientations, as spheres are rotation invariant

			auto res = mieCalc::calc(params);

			Qsca += res->qsca;
			Qext += res->qext;
			Qbk += res->qbk;
			g += res->g;

			// Just for compatibility
			auto angres = mieAngleRes::calc(res, 180);
			data.miedata.push_back(angres);
			
			stats->stats["Qbk"] = Qbk;
			stats->stats["Qext"] = Qext;
			stats->stats["Qsca"] = Qsca;
			stats->stats["g"] = g;

			out << temp << "," << freq << "," << aeff << ","
				<< m.real() << "," << m.imag() << "," << vfrac << "," << sizep << ","
				<< Qbk << "," << Qext << "," << Qsca << "," << g << endl;
			
			if (noxml == false)
				::serialization::write(data, string(ofile).append(".xml"));
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



