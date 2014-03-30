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
#include <fstream>
#include <ios>
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
#include "../../rtmath/rtmath/plugin.h"
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
			("input,i", po::value< vector<string> >()->multitoken(), "Input shape file(s)")
			("output,o", po::value<vector<string> >(), "Output file(s). Each input is written to all of the outputs.")
			("export-type", po::value<string>(), "Identifier to export (i.e. ar_rot_data)")
			("export,e", po::value<string>(), "Export filename (all shapes are combined into this)")
			//("convex-hull,c", "Calculate and write convex hull")
			;
		rtmath::debug::add_options(cmdline, config, hidden);
		rtmath::ddscat::stats::shapeFileStats::add_options(cmdline, config, hidden);

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);
		rtmath::ddscat::stats::shapeFileStats::process_static_options(vm);

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

		if (!vm.count("input"))
			doHelp("Need to specify input file(s).");
		vector<string> inputs = vm["input"].as< vector<string> >();

		//if (!vm.count("output"))
		//	doHelp("Need to specify output file(s).");
		vector<string> output;
		if (vm.count("output"))
			output = vm["output"].as< vector<string> >();

		bool doExport = false;
		std::string exportType, exportFilename;
		if (vm.count("export"))
		{
			doExport = true;
			exportType = vm["export-type"].as<string>();
			exportFilename = vm["export"].as<string>();
			cerr << "Exporting to: " << exportFilename << " with format " << exportType << endl;
		}

		std::vector<std::shared_ptr<rtmath::registry::IOhandler> > outputios(output.size());
		std::shared_ptr<rtmath::registry::IOhandler> outputexp;

		for (const auto &input : inputs)
		{
			// Validate input files
			path pi(input);
			if (!exists(pi))
				throw rtmath::debug::xMissingFile(input.c_str());
			if (is_directory(pi))
				throw rtmath::debug::xPathExistsWrongType(input.c_str());

			using rtmath::ddscat::shapefile::shapefile;
			using rtmath::ddscat::stats::shapeFileStats;
			boost::shared_ptr<shapefile> shp;
			boost::shared_ptr<shapeFileStats> stats;
			// Load the shape file
			try {

				using namespace rtmath::Voronoi;
				shp = boost::shared_ptr<shapefile>(new shapefile(input));
				stats = shapeFileStats::genStats(shp);
				// stats already generates this
				auto vd = shp->generateVoronoi("standard", VoronoiDiagram::generateStandard);
				auto cvxCands = vd->calcCandidateConvexHullPoints();
				shp->latticeExtras["cvxCands"] = cvxCands;
				auto SAfracExternal = vd->calcPointsSAfracExternal();
				shp->latticeExtras["SAfracExternal"] = SAfracExternal;
				auto depth = vd->calcSurfaceDepth();
				shp->latticeExtras["SurfaceDepth"] = depth;
				auto depthSrc = vd->calcSurfaceDepthVectors();
				shp->latticeExtras["SurfaceDepthVectors"] = depthSrc;
				auto depthNeighs = vd->calcSurfaceNumNeighs();
				shp->latticeExtras["SurfaceDepthNNeighs"] = depthNeighs;
				auto depthOrder = vd->calcSurfaceFillingOrder();
				shp->latticeExtras["SurfaceFillingOrder"] = depthOrder;

				for (size_t i = 0; i < output.size(); ++i)
				{
					try {
						auto opts = registry::IO_options::generate();
						opts->filename(output[i]);

						if (output[i].size())
							outputios[i] = stats->writeMulti(outputios[i], opts);
					}
					catch (rtmath::debug::xUnknownFileFormat &e)
					{
						std::cerr << "Error: unknown file format when writing " << output[i] << endl;
						std::cerr << e.what() << endl;
					}
				}
				if (doExport) /// \todo Figure out how to do multiple exports (command-line parsing)
				{
					auto opts = rtmath::registry::IO_options::generate();
					opts->filename(exportFilename);
					opts->exportType(exportType);
					try {
						if (vd->canWriteMulti(outputexp, opts))
							outputexp = vd->writeMulti(outputexp, opts);
						else if (stats->canWriteMulti(outputexp, opts))
							outputexp = stats->writeMulti(outputexp, opts);
						else if (shp->canWriteMulti(outputexp, opts))
							outputexp = shp->writeMulti(outputexp, opts);
					} catch (rtmath::debug::xUnknownFileFormat &e) {
							std::cerr << "Error: unknown file format when writing " << exportFilename << endl;
							std::cerr << e.what() << endl;
					}
				}
			}
			catch (std::exception &e)
			{
				std::cerr << "Error processing the file: " << input << endl;
				std::cerr << e.what() << std::endl;
				continue;
			}
			catch (...)
			{
				std::cerr << "Error processing the file: " << input << endl;
				continue;
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

