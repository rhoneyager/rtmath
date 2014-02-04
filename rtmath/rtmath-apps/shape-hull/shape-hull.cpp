/* shape-hull
* This program is designed to take a shapefile and write a vtk file corresponding 
* to either the initial points or the convex hull.
*/
#pragma warning( push )
#pragma warning( disable : 4996 ) // Dumb boost uuid warning
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

#include <Ryan_Debug/debug.h>
#include <Ryan_Serialization/serialization.h>
#pragma warning( pop ) 

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/config.h"
#include "../../rtmath/rtmath/hash.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/ddscat/hulls.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/registry.h"
#include "../../rtmath/rtmath/error/debug.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-shape-hull\n\n";

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", 1);
		p.add("output", 2);

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		//ddscat::shapeFileStats::add_options(cmdline, config, hidden);
		//Ryan_Serialization::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value< string >(), "input shape file")
			("output,o", po::value<vector<string> >(), "Output file(s)")
			//("convex-hull,c", "Calculate and write convex hull")
			;
		rtmath::debug::add_options(cmdline, config, hidden);

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);


		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << endl;
			cerr << message << endl;
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");

		//Ryan_Serialization::process_static_options(vm);
		//ddscat::shapeFileStats::process_static_options(vm);

		bool convex = false;
		if (vm.count("convex-hull")) convex = true;

		string input = vm["input"].as< string >();
		if (vm.count("input"))
			cerr << "Input file is: " << input << endl;
		else doHelp("Need to specify an input file");

		vector<string> output = vm["output"].as< vector<string> >();
		if (!vm.count("output"))
			doHelp("Need to specify output file(s).");

		// Validate input files
		path pi(input);
		if (!exists(pi)) 
			throw rtmath::debug::xMissingFile(input.c_str());
		if (is_directory(pi)) 
			throw rtmath::debug::xPathExistsWrongType(input.c_str());

		// Load the shape file
		rtmath::ddscat::shapefile::shapefile shp(input);
		using namespace rtmath::Voronoi;
		auto vd = VoronoiDiagram::generateStandard(shp.mins, shp.maxs, shp.latticePts);
		auto cvxCands = vd->calcCandidateConvexHullPoints();
		shp.latticeExtras["cvxCands"] = cvxCands;
		auto depth = vd->calcSurfaceDepth();
		shp.latticeExtras["SurfaceDepth"] = depth; //.col(3);

		for (const auto& outfile : output)
			shp.write(outfile);
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

