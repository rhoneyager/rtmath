/** Program exists to iterate over shape files in a directory,
 * recover their stats, and select those fitting the desired
 * max dimension range. The matching shape have their directories
 * symlinked into the output.
 **/
#pragma warning( push )
#pragma warning( disable : 4996 ) // Dumb boost uuid warning
#pragma warning( disable : 4800 ) // forcing non-bool type to true or false
#include <cmath>
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <set>

#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/serialization/vector.hpp>

#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/fs.h>
#include <Ryan_Debug/splitSet.h>
#pragma warning( pop ) 

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"


double dSpacing = 0;

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-shape-hogan-select\n\n";

		namespace po = boost::program_options;

		po::positional_options_description p;

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		ddscat::stats::shapeFileStats::add_options(cmdline, config, hidden);
		//Ryan_Serialization::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("dipole-spacing,d", po::value<double>(), "Set dipole spacing.")
			("input,i", po::value< vector<string> >()->multitoken(), "Input directories to test. Non-recursive.")

			("output,o", po::value<string>(), "Output directory")
			("min-md", po::value<double>()->default_value(0), "Minimum max dimension (mm)")
			("max-md", po::value<double>()->default_value(0), "Maximum max dimension (mm)")
			;

		rtmath::debug::add_options(cmdline, config, hidden);

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		auto doHelp = [&](const std::string &m)
		{
			std::cerr << desc << "\n" << m << std::endl;
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");
		rtmath::debug::process_static_options(vm);
		ddscat::stats::shapeFileStats::process_static_options(vm);

		if (!vm.count("dipole-spacing")) doHelp("Must specify dipole spacing");
		dSpacing = vm["dipole-spacing"].as<double>();

		double minmd = vm["min-md"].as<double>();
		double maxmd = vm["max-md"].as<double>();

		vector<string> inputs;
		if (vm.count("input"))
		{
			inputs = vm["input"].as< vector<string> >();
			cerr << "Input directories are:" << endl;
			for (auto it = inputs.begin(); it != inputs.end(); it++)
				cerr << "\t" << *it << "\n";
		} else doHelp("Need to specify input directories.");

		string output;
		if (vm.count("output"))
		{
			output = vm["output"].as<string>(); 
			cerr << "Outputting to: " << output << endl;
		} else doHelp("Must specify an output directory.");
		path pout(output);
		if (!exists(pout)) boost::filesystem::create_directory(pout);
		path pouts = Ryan_Debug::fs::expandSymlink<path,path>(pout);
		if (!exists(pouts)) boost::filesystem::create_directory(pouts);
		if (!is_directory(pouts)) RDthrow(Ryan_Debug::error::xPathExistsWrongType())
			<< Ryan_Debug::error::file_name(output)
			<< Ryan_Debug::error::otherErrorText("Output is not a directory");

		using namespace rtmath::ddscat::shapefile;
		vector<string> vinputs;
		for (auto it = inputs.begin(); it != inputs.end(); it++)
		{
			cerr << "Processing " << *it << endl;
			path pi(*it);
			if (!exists(pi)) RDthrow(Ryan_Debug::error::xMissingFile())
				<< Ryan_Debug::error::file_name(*it);
			// Expand any symlinks
			path pis = Ryan_Debug::fs::expandSymlink<path,path>(pi);
			if (!exists(pi)) RDthrow(Ryan_Debug::error::xMissingFile())
				<< Ryan_Debug::error::file_name(*it);
			if (!is_directory(pis)) {
				cerr << "\tNot a directory!" << endl;
				continue;
			}
			path ps = pis / "shape.dat";
			if (!exists(ps)) {
				cerr << "\tNo shape.dat found for " << ps << endl;
				continue;
			}
			path pavg = pis / "w000r000.avg";
			if (!exists(pavg)) {
				cerr << "\tNo w000r000.avg found at " << pavg << endl;
				continue;
			}
			path ppar = pis / "ddscat.par";
			if (!exists(ppar)) {
				cerr << "\tNo ddscat.par found at " << ppar << endl;
				continue;
			}
			cerr << "\tValid run." << endl;
			try {
				// Needed to retreive statistics
				boost::shared_ptr<rtmath::ddscat::shapefile::shapefile> shp;
				shp = rtmath::ddscat::shapefile::shapefile::generate(ps.string());
				cerr << "\tThis is shape " << shp->hash().lower << endl;
				auto stats = rtmath::ddscat::stats::shapeFileStats::genStats(shp);
				if (!stats) { cerr << " Unable to generate stats" << endl; continue; }

				double aeff_um = 0, md_mm = 0;
				aeff_um = stats->aeff_dipoles_const * dSpacing;
				md_mm = stats->max_distance * dSpacing / 1000;
				size_t nPoints = shp->numPoints;
				// Check if max dimension is in range
				if (minmd && (md_mm < minmd)) continue;
				if (maxmd && (md_mm > maxmd)) continue;

				// Determine output folder
				string outnpoints = boost::lexical_cast<string>(nPoints);
				size_t index = 0;
				path poutf = pouts / path(outnpoints);
				while (exists(poutf)) {
					index++;
					string ofld = outnpoints;
					ofld.append("_");
					ofld.append(boost::lexical_cast<string>(index));
					poutf = pouts / path(ofld);
				}
				// poutf is the correct folder.
				// Hard link ddscat.par, shape.dat and w000r000.avg.
				boost::filesystem::create_directory(poutf);
				create_hard_link(ps, poutf / "shape.dat");
				create_hard_link(pavg, poutf / "w000r000.avg");
				create_hard_link(ppar, poutf / "ddscat.par");
			} catch (std::exception &e) {
				cerr << e.what() << endl;
			}
		}
	}
	catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

