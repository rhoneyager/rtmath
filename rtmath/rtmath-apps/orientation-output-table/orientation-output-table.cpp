/* orientation-output-table
 * This program will calculate 3d weights on a cyclic distribution of angles, 
 * and uses this data to reweight a ddscat output. 
 */
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
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
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
		cerr << "rtmath-orientation-output-table\n\n";

		namespace po = boost::program_options;

		po::positional_options_description p;
		//p.add("input", -1);
		//p.add("output", 2);

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		//ddscat::shapeFileStats::add_options(cmdline, config, hidden);
		//Ryan_Serialization::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value<string >(), "input ddOutput file or ddscat output directory")
			("etas,e", po::value<string>()->default_value("0"), "Specify nadir angle intervals.")
			("mean_theta", po::value<double>()->default_value(0),
			"Base theta mean (degrees)")
			("mean_phi", po::value<double>()->default_value(0),
			"Base phi mean (degrees)")
			("kappa", po::value<string>()->default_value("10"),
			"Kappa (degrees) is comparable to 1/sigma^2. Limit of infinity for uniform distribution, "
			"and 0 for only the mean.")
			("output,o", po::value<string>(), "Output weight file")
			("method", po::value<string>()->default_value("vMFdual"), "Specify the method used in the orientation "
			"calculations (DDSCAT, vMF, vMFdual).");
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

		std::string setas;

		if (vm.count("etas"))
			setas = vm["etas"].as<std::string>();
		std::set<double> etas;
		rtmath::config::splitSet(setas, etas);
		
		rtmath::ddscat::rotations rot;
		
		if (!vm.count("output")) doHelp("Need to specify an output file.");
		if (!vm.count("input")) doHelp("Need to specify an input file.");
		std::string file = vm["input"].as<std::string >();

		auto expandSymlinks = [](const boost::filesystem::path &p) -> boost::filesystem::path
		{
			using namespace boost::filesystem;
			if (is_symlink(p))
			{
				path pf = boost::filesystem::absolute(read_symlink(p), p.parent_path());
				return pf;
			} else {
				return p;
			}
		};
		boost::filesystem::path pInput(file);
		path ps = expandSymlinks(pInput);
		
		using namespace rtmath::ddscat;
		boost::shared_ptr<ddOutput> ddOut;
		if (is_directory(ps))
		{
			// Input is a ddscat run
			boost::shared_ptr<ddOutput> ddOut;
			ddOut = ddOutput::generate(ps.string());
		} else if (Ryan_Serialization::known_format(ps)) {
			// Input may be a ddOutput file
			// Read will fail if it is not the right file type
			ddOut = boost::shared_ptr<ddOutput>(new ddOutput);
			ddOut->readFile(ps.string());
		} else doHelp("Unable to parse input expression.");

		ddOut->parfile->getRots(rot);
		
		using namespace rtmath::ddscat::weights;
		ddWeightsDDSCAT dw(rot);

		double muT = vm["mean_theta"].as<double>();
		double muP = vm["mean_phi"].as<double>();
		std::string skappas = vm["kappa"].as<std::string>();
		std::set<double> kappas;
		rtmath::config::splitSet(skappas, kappas);

		std::string method;
		method = vm["method"].as<std::string>();
		std::transform(method.begin(), method.end(), method.begin(), ::tolower);

		std::string sofile = vm["output"].as<string>();
		ofstream out(sofile.c_str());

		out << "Method\tShape Hash\teta\tkappa\tmu_theta\tmu_phi\tFrequency (GHz)\tAeff (um)\t"
			"Qsca_m\tQbk_m\n";

		for (auto &kappa : kappas)
		for (auto &eta : etas)
		{
			boost::shared_ptr<OrientationWeights3d> ow;
			if(method == "vmf")
			{
				ow = boost::shared_ptr<OrientationWeights3d> (new VonMisesFisherWeights(
					dw, muT+eta, muP, kappa));
			} else if (method == "vmfdual")
			{
				ow = boost::shared_ptr<OrientationWeights3d> (new BimodalVonMisesFisherWeights(
					dw, muT+eta, muP, kappa));
			} else if (method == "ddscat")
			{
				ow = boost::shared_ptr<OrientationWeights3d> (new DDSCAT3dWeights(dw));
			} else doHelp("Unknown weighting method");

			
			OrientationWeights3d::weightTable wts;
			ow->getWeights(wts);
			double Qsca = 0, Qbk = 0;
			
			for (auto it = wts.cbegin(); it != wts.cend(); ++it)
			{
				// Find the appropriate sca file
				/// \todo Rewrite to add weighting table to ddOutput better.
				auto ot = std::find_if(ddOut->scas.cbegin(), ddOut->scas.cend(),
					[&](const boost::shared_ptr<ddOutputSingle> &val)
				{
					if (val->beta() < it->at(IntervalTable3dDefs::BETA_MIN)) return false;
					if (val->beta() >= it->at(IntervalTable3dDefs::BETA_MAX)) return false;
					if (val->theta() < it->at(IntervalTable3dDefs::THETA_MIN)) return false;
					if (val->theta() >= it->at(IntervalTable3dDefs::THETA_MAX)) return false;
					if (val->phi() < it->at(IntervalTable3dDefs::PHI_MIN)) return false;
					if (val->phi() >= it->at(IntervalTable3dDefs::PHI_PIVOT)) return false;
					return true;
				});
				if (ot == ddOut->scas.cend())
				{
					cerr << "Could not match rotation ("
						<< it->at(IntervalTable3dDefs::BETA_PIVOT) << ", "
						<< it->at(IntervalTable3dDefs::THETA_PIVOT) << ", "
						<< it->at(IntervalTable3dDefs::PHI_PIVOT) << ").\n";
					continue;
				}

				Qsca += (*ot)->getStatEntry(QSCAM) * it->at(IntervalTable3dDefs::WEIGHT);
				Qbk += (*ot)->getStatEntry(QBKM) * it->at(IntervalTable3dDefs::WEIGHT);
			}

			out << method << "\t"
				<< ddOut->shapeHash.lower << "\t"
				<< eta << "\t"
				<< kappa << "\t"
				<< muT << "\t"
				<< muP << "\t"
				<< ddOut->freq << "\t"
				<< ddOut->aeff << "\t"
				<< Qsca << "\t"
				<< Qbk << "\n";

		}

	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

