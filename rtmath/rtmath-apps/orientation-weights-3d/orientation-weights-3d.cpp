/* orientation-weights-3d
 * This program will calculate 3d weights on a cyclic distribution of angles. */
#pragma warning( push )
#pragma warning( disable : 4800 ) // forcing non-bool type to true or false
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <set>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/math/constants/constants.hpp>

#include <Ryan_Debug/debug.h>
#include <Ryan_Serialization/serialization.h>
#pragma warning( pop ) 

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/ddscat/ddweights.h"
#include "../../rtmath/rtmath/error/debug.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	const double pi = boost::math::constants::pi<double>();

	try {
		cerr << "rtmath-orientation-weights-3d\n\n";

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);
		//p.add("output", 2);

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		//ddscat::shapeFileStats::add_options(cmdline, config, hidden);
		//Ryan_Serialization::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value<string >(), "input ddPar file")
			("betas,b", po::value<string >(), "Specify beta rotations")
			("thetas,t", po::value<string > (), "Specify theta rotations")
			("phis,p", po::value<string >(), "Specify phi rotations")
			("mean_theta", po::value<double>()->default_value(0),
			"Theta mean (degrees)")
			("mean_phi", po::value<double>()->default_value(0),
			"Phi mean (degrees)")
			("kappa", po::value<double>()->default_value(10),
			"Kappa (degrees) is comparable to 1/sigma^2. Limit of infinity for uniform distribution, "
			"and 0 for only the mean.")
			("output,o", po::value<string>(), "Output weight file")
			;

		hidden.add_options()
			;

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << endl;
			cerr << message << endl;
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");

		//Ryan_Serialization::process_static_options(vm);
		//ddscat::shapeFileStats::process_static_options(vm);

		std::string sbetas, sthetas, sphis;

		if (vm.count("betas"))
			sbetas = vm["betas"].as<std::string>();
		if (vm.count("thetas"))
			sthetas = vm["thetas"].as<std::string>();
		if (vm.count("phis"))
			sphis = vm["phis"].as<std::string>();

		rtmath::ddscat::rotations rot;
		// Also load in any ddscat.par files for recombination
		if (vm.count("input"))
		{
			std::string file = vm["input"].as<std::string >();
			rtmath::ddscat::ddPar par(file);
			par.getRots(rot);
		} else {
			// interval is a dummy variable
			double bMin, bMax, tMin, tMax, pMin, pMax, interval;
			std::string specializer;
			size_t nB, nT, nP;
			rtmath::config::extractInterval(sbetas, bMin, bMax,interval, nB, specializer);
			if (specializer != "lin") 
				throw debug::xBadInput("Beta interval needs to be linearly spaced.");
			rtmath::config::extractInterval(sthetas, tMin, tMax,interval, nT, specializer);
			if (specializer != "cos") 
				throw debug::xBadInput("Theta interval needs to be cosine spaced.");
			rtmath::config::extractInterval(sphis, pMin, pMax,interval, nP, specializer);
			if (specializer != "lin") 
				throw debug::xBadInput("phi interval needs to be linearly spaced.");

			rot = rtmath::ddscat::rotations(bMin,bMax,nB,tMin,tMax,nT,pMin,pMax,nP);
		}
		
		using namespace rtmath::ddscat::weights;
		ddWeightsDDSCAT dw(rot);

		double muT = vm["mean_theta"].as<double>();
		double muP = vm["mean_phi"].as<double>();
		double kappa = vm["kappa"].as<double>();

		std::string method;
		method = "vmf";
		//method = vm["method"].as<std::string>();
		std::transform(method.begin(), method.end(), method.begin(), ::tolower);

		boost::shared_ptr<OrientationWeights3d> ow;
		if(method == "vmf")
		{
			ow = boost::shared_ptr<OrientationWeights3d> (new VonMisesFisherWeights(
				dw, muT, muP, kappa));
		} else doHelp("Unknown weighting method");


		if (vm.count("output"))
		{
			std::string sofile = vm["output"].as<string>();
			ofstream out(sofile.c_str());
			out << "Weightings ( " << method << ", muT = " << muT << ", muP = " << muP << ", kappa = " << kappa << " )" << endl;
			out << "Beta_min (degrees)\tBeta_max (degrees)\tBeta Pivot (degrees)\t"
				"Theta_min (degrees)\tTheta_max (degrees)\tTheta Pivot (degrees)\t"
				"Phi_min (degrees)\tPhi_max (Degrees)\tPhi Pivot (degrees)\t"
				"PDF\tCDF\n";
			
			OrientationWeights3d::weightTable wts;
			ow->getWeights(wts);
			double cdf = 0;
			for (auto it = wts.cbegin(); it != wts.cend(); ++it)
			{
				cdf += it->at(IntervalTable3dDefs::WEIGHT);
				out << it->at(IntervalTable3dDefs::BETA_MIN) << "\t"
					<< it->at(IntervalTable3dDefs::BETA_MAX) << "\t"
					<< it->at(IntervalTable3dDefs::BETA_PIVOT) << "\t"
					<< it->at(IntervalTable3dDefs::THETA_MIN) << "\t"
					<< it->at(IntervalTable3dDefs::THETA_MAX) << "\t"
					<< it->at(IntervalTable3dDefs::THETA_PIVOT) << "\t"
					<< it->at(IntervalTable3dDefs::PHI_MIN) << "\t"
					<< it->at(IntervalTable3dDefs::PHI_MAX) << "\t"
					<< it->at(IntervalTable3dDefs::PHI_PIVOT) << "\t"
					<< it->at(IntervalTable3dDefs::WEIGHT) << "\t"
					<< cdf << "\n";
			}
		}

	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

