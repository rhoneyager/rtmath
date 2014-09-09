/* shape-perturb-interior
* This program takes a shapefile, isolates the interior, and scrambles it!
*/
#pragma warning( push )
#pragma warning( disable : 4996 ) // Dumb boost uuid warning
#pragma warning( disable : 4800 ) // forcing non-bool type to true or false
#include <cmath>
#include <algorithm>	// std::random_shuffle
#include <ctime>        // std::time
#include <cstdlib>      // std::rand, std::srand
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <Ryan_Debug/debug.h>
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
		cerr << "rtmath-shape-perturb-interior\n\n";

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
			("input,i", po::value<string>(), "Input shape file")
			("output,o", po::value<vector<string> >(), "Output files")

			("output-dielectric", po::value<string>()->default_value("dielInterior.tab"), 
			 "Output effective dielectric file for interior "
			 "(if interior uniformization is used)")
			("frequency,f", po::value<string>(), "List of frequencies (default of GHz). Can also take rtmath.conf-provided frequency key (ex: GPM_freqs).")
			("temperature,T", po::value<string>(), "List of temperatures (K)")
			("method", po::value<string>()->default_value("Maxwell-Garnett-Ellipsoids"), "Method used to calculate the resulting dielectric "
			"(Sihvola, Debye, Maxwell-Garnett-Spheres, Maxwell-Garnett-Ellipsoids). "
			"Only matters if volume fractions are given. Then, default is Sihvola.")

			("threshold", po::value<int>()->default_value(3), "All dipole sites greater than or equal to "
			"this threshold are subject to randomization.")
			("use-effective-dielectric", "If specified, then all interior points will be replaced by "
			 "an effective dielectric (scaled based on filled vs. total volume fraction. ")

			("update-db", "Insert shape file entries into database")
			("tag", po::value<vector<string> >()->multitoken(), "Using \"key=value pairs\", add tags to the output (not with .shp files)")
			("flake-type", po::value<string>(), "Specify flake type (e.g. oblate, oblate_small, prolate, ...")
			("list-hash", "Write the hashes of each processed shapefile to stdout (for scripting)")
			("hash-shape", "Store shapefile hash")
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
		//rtmath::plugins::internal::tsv::registerTSV();


		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << endl;
			cerr << message << endl;
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");


		if (!vm.count("input"))
			doHelp("Need to specify input file(s).");
		string input = vm["input"].as< string >();

		if (!vm.count("output"))
			doHelp("Need to specify output file(s).");
		string output = vm["output"].as< string >();

		int threshold = vm["threshold"].as<int>();
		bool useEffectiveDiel = false;
		if (vm.count("use-effective-dieelctric")) useEffectiveDiel = true;
		string outputDiel = vm["output-dielectric"].as<string>();

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

		using namespace rtmath::Voronoi;
		shp = boost::shared_ptr<shapefile> (new shapefile(input));
		auto vd = shp->generateVoronoi("standard", VoronoiDiagram::generateStandard);

		auto depthSfc = vd->calcSurfaceDepth();
		auto cellmap = vd->getCellMap();

		// Preserve the dipoles that make up the 'surface'
		boost::shared_ptr<Eigen::MatrixXi > resPts(new Eigen::MatrixXi());
		resPts->resize(shp->numPoints,3);
		size_t outerCount = 0; // Keeps track of the number of points on the 'outside'.

		// Keep track of the inner volume fraction
		size_t innerPointsTotal = 0;
		size_t innerPointsFilled = 0;

		// Also track all potential dipole sites
		std::vector<Eigen::Matrix3i> potentialInnerPoints;
		potentialInnerPoints.reserve(cellmap->rows());

		// First, iterate over all of the entries in depthSfc to extract the constant surface points.
		for (int i=0; i<depthSfc->rows(); ++i)
		{
			// First three arguments are the coordinate. Fourth is the surface depth.
			auto crds = depthSfc->block<1,3>(i,0);
			int depth = (int) (*depthSfc)(i,3);
			if (depth < threshold)
			{
				resPts->block<1,3>(outerCount,0) = crds.cast<int>();
				outerCount++;
			} else innerPointsFilled++;
		}

		// Iterate over all entries in the cell map, while consulting the surface depth field.
		for (int i=0; i<cellmap->rows(); ++i)
		{
			// i is the probe point id
			// The first three columns are the probe point coordinates
			// The fourth column is the the voronoi point id (row)
			auto crds = cellmap->block<1,3>(i,0);
			int voroid = (*cellmap)(i,3);
			int depth = (int) (*depthSfc)(voroid,3);
			if (depth >= threshold)
			{
				innerPointsTotal++;
				Eigen::Matrix3i p;
				p.block<1,3>(0,0) = crds.block<1,3>(0,0);
				potentialInnerPoints.push_back(std::move(p));
			}
		}

		cout << "There are " << shp->numPoints << " total.\n"
			<< "The threshold is " << threshold << ".\n"
			<< "The surface has " << outerCount << " points.\n"
			<< "The interior has " << innerPointsFilled << " filled points, "
			"and " << innerPointsTotal << " total potential sites.\n"
			<< "The inner fraction is " << 100.f * (float) innerPointsFilled 
			/ (float) innerPointsTotal << " percent." << std::endl;

		// Scramble the potential inner points listing.
		//auto myrandom = [](int i) {return std::rand()%i;};
		std::srand ( unsigned ( std::time(0) ) );
		std::random_shuffle ( potentialInnerPoints.begin(), potentialInnerPoints.end() );
		// After shuffling, take the first innerPointsFilled and write them to the resultant shape.
		for (size_t i=0; i < (size_t) innerPointsFilled; ++i)
		{
			resPts->block<1,3>(outerCount+i, 0) = potentialInnerPoints[i].block<1,3>(0,0);
		}

		// All points are now filled. Write a shape file from the data.
		// Not using the standard ddscat class at first, as it has to be read back in.
		std::ostringstream out;
		out << shp->desc << " - randomized with threshold " << threshold << "\n";
		out << shp->numPoints << std::endl;
		out << "1.0 0.0 0.0\n0.0 1.0 0.0\n1 1 1\n0 0 0\n";
		out << "No. ix iy iz dx dy dz\n";
		for (size_t i=0; i<shp->numPoints; ++i)
			out << i+1 << "\t" << (*resPts)(i,0) << "\t" << (*resPts)(i,1) << "\t" << 
			(*resPts)(i,2) << "\t1\t1\t1\n";
		std::string ostr = out.str();
		std::istringstream in(ostr);
		shapefile oshp(in);
		oshp.fixStats();
		oshp.write(output);


	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}
