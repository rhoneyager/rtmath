/* orientation-weights-1d
 * This program will calculate 1d weights on a cyclic distribution of angles. */
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
#include <Ryan_Debug/error.h>
#include <Ryan_Debug/splitSet.h>
//#include <Ryan_Serialization/serialization.h>
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
		cerr << "rtmath-orientation-weights-1d\n\n";

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
			("rotations,r", po::value<string >(), "Specify rotations")
			("method,m", po::value<string>()->default_value("VonMises"),
			"Specify rotation weight generation method (uniform, vonmises). Capitalization is ignored.")
			("mean", po::value<double>()->default_value(0),
			"Mean (degrees) for weighting schemes that have a mean.")
			("kappa", po::value<double>()->default_value(10),
			"Kappa (degrees) for weighting schemes that take this parameter.")
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

		double mu = vm["mean"].as<double>();
		double kappa = vm["kappa"].as<double>();

		std::string srots, method;
		method = vm["method"].as<std::string>();
		std::transform(method.begin(), method.end(), method.begin(), ::tolower);

		if (vm.count("rotations"))
			srots = vm["rotations"].as<std::string>();

		// interval is a dummy variable

		using namespace rtmath::ddscat::weights;
		double Min, Max, interval;
		std::string specializer;
		size_t N;
		Ryan_Debug::splitSet::extractInterval(srots, Min, Max,interval, N, specializer);
		if (specializer != "lin" && specializer != "cos") 
			RDthrow(Ryan_Debug::error::xBadInput()) << Ryan_Debug::error::otherErrorText("Interval needs to be linearly or cosine spaced.");
		
		boost::shared_ptr<ddWeights> dw;
		if (specializer == "lin")
		{
			dw = boost::shared_ptr<ddWeights>(new ddWeightsLinInt (Min, Max, N));
		} else if (specializer == "cos")
		{
			dw = boost::shared_ptr<ddWeights>(new ddWeightsCosInt (Min, Max, N));
		} else doHelp("Unhandled interval type.");

		boost::shared_ptr<OrientationWeights1d> ow;
		if (method == "uniform")
		{
			ow = boost::shared_ptr<OrientationWeights1d>(new Uniform1dWeights(*dw));
		} else if (method == "vonmises")
		{
			ow = boost::shared_ptr<OrientationWeights1d>(new VonMisesWeights(*dw,mu,kappa));
		} else doHelp("Unknown weighting method");

		std::ostream *out = &(std::cout);
		std::ofstream *oof = nullptr;
		if (vm.count("output"))
		{
			std::string sofile = vm["output"].as<string>();
			oof = new std::ofstream(sofile.c_str());
			out = oof;
		}

		{
			*out << "Weightings ( " << method << ", mu = " << mu << ", kappa = " << kappa << " ) for " << Min << ":" << N << ":" << Max << ":" << specializer << endl;
			*out << "Angle midpoint (Degrees)\tAngle low (degrees)\tAngle high (degrees)\tPDF\tCDF\n";

			OrientationWeights1d::weightTable wts;
			ow->getWeights(wts);
			double cdf = 0;
			for (auto it = wts.cbegin(); it != wts.cend(); ++it)
			{
				cdf += it->second;
				double low, high, pivot;
				dw->interval(it->first, low, high, pivot);

				*out << pivot<< "\t" << low << "\t" << high << "\t" << it->second << "\t" << cdf << endl;
			}
		}

	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

