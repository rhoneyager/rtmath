/* orientation-weights-ddscat
 * This program will calculate the weights for different orientations of a ddOutput object or 
 * a set of ddPar objects. */
#pragma warning( push )
#pragma warning( disable : 4800 ) // forcing non-bool type to true or false
#include <cmath>
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <set>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/math/constants/constants.hpp>

#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/splitSet.h>
#pragma warning( pop ) 

#include "../../rtmath/rtmath/common_templates.h"
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
		cerr << "rtmath-orientation-weights-ddscat\n\n";

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
			auto par = rtmath::ddscat::ddPar::generate(file);
			par->getRots(rot);
		} else {
			// interval is a dummy variable
			double bMin, bMax, tMin, tMax, pMin, pMax, interval;
			std::string specializer;
			size_t nB, nT, nP;
			Ryan_Debug::splitSet::extractInterval(sbetas, bMin, bMax,interval, nB, specializer);
			if (specializer != "lin") 
				RDthrow(Ryan_Debug::error::xBadInput()) << Ryan_Debug::error::otherErrorText("Beta interval needs to be linearly spaced.");
			Ryan_Debug::splitSet::extractInterval(sthetas, tMin, tMax,interval, nT, specializer);
			if (specializer != "cos") 
				RDthrow(Ryan_Debug::error::xBadInput()) << Ryan_Debug::error::otherErrorText("Theta interval needs to be cosine spaced.");
			Ryan_Debug::splitSet::extractInterval(sphis, pMin, pMax,interval, nP, specializer);
			if (specializer != "lin") 
				RDthrow(Ryan_Debug::error::xBadInput()) << Ryan_Debug::error::otherErrorText("phi interval needs to be linearly spaced.");

			rot = rtmath::ddscat::rotations(bMin,bMax,nB,tMin,tMax,nT,pMin,pMax,nP);
		}

		rtmath::ddscat::weights::ddWeightsDDSCAT ddw(rot);
		rtmath::ddscat::weights::ddWeightsLinInt ddbeta(rot.bMin(), rot.bMax(), rot.bN());

		std::ostream *out = &(std::cout);
		std::ofstream *oof = nullptr;

		if (vm.count("output"))
		{
			std::string sofile = vm["output"].as<string>();
			oof = new std::ofstream(sofile.c_str());
			out = oof;
		}


		{
			*out << "Weightings for " 
				"beta: " << rot.bMin() << ":" << rot.bN() << ":" << rot.bMax() << ":lin "
				"theta: " << rot.tMin() << ":" << rot.tN() << ":" << rot.tMax() << ":cos "
				"phi: " << rot.pMin() << ":" << rot.pN() << ":" << rot.pMax() << ":lin\n";
			*out << "Theta\tPhi\tBeta\tWa1\tWa2\tWeight\tCDF\n";

			set<double> betas, thetas, phis;
			rot.betas(betas);
			rot.thetas(thetas);
			rot.phis(phis);
			double sumTotal = 0;
			for (auto &theta : thetas)
			for (auto &phi : phis)
			for (auto &beta : betas)
					{
						double totWeight = ddw.getWeight(beta,theta,phi);
						sumTotal += totWeight;
						double bWeight = ddbeta.weightBase(beta);
						double a1weight = totWeight / bWeight;
						*out << theta << "\t" << phi << "\t" << beta << "\t" << 
							a1weight << "\t" << bWeight << "\t" << totWeight << "\t" << sumTotal << endl;
					}
			*out << endl;
			if (abs(sumTotal-1.0) > 0.001) RDthrow(Ryan_Debug::error::xAssert()) << Ryan_Debug::error::otherErrorText("Weights do not sum to unity!");
			//out << "Sums:\t\t\t\t" << sumTotal << endl << endl;
		}

	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

