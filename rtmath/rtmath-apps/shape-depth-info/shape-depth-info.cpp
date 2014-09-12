/* shape-depth-interior
* This program takes a shapefile and reports volume fractions at 
* different depths within the object.
*/
#pragma warning( push )
#pragma warning( disable : 4996 ) // Dumb boost uuid warning
#pragma warning( disable : 4800 ) // forcing non-bool type to true or false
#include <cmath>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <Ryan_Debug/debug.h>
#pragma warning( pop ) 

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/config.h"
#include "../../rtmath/rtmath/hash.h"
#include "../../rtmath/rtmath/units.h"
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
		cerr << "rtmath-shape-depth-info\n\n";

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
			("input,i", po::value<vector<string> >(), "Input shape file(s)")
			("output,o", po::value<string >(), "Output file (tsv format)")

			("use-db", po::value<bool>()->default_value(true), "Use database to supplement loaded information")
			("from-db", po::value<bool>()->default_value(false), "Perform search on database and select files matching criteria.")
			("match-hash", po::value<vector<string> >()->multitoken(), "Match lower hashes")
			("match-flake-type", po::value<vector<string> >()->multitoken(), "Match flake types")
			("match-dipole-spacing", po::value<vector<float> >()->multitoken(), "Match typical dipole spacings")
			("match-dipole-numbers", po::value<vector<size_t> >()->multitoken(), "Match typical dipole numbers")
			("match-parent-flake-hash", po::value<vector<string> >()->multitoken(), "Match flakes having a given parent hash")
			("match-parent-flake", "Select the parent flakes")

			//("threshold", po::value<int>()->default_value(3), "All dipole sites greater than or equal to "
			//"this threshold are subject to randomization.")
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


		vector<string> inputs;
		if (vm.count("input"))
		{
			inputs = vm["input"].as< vector<string> >();
			cerr << "Input files are:" << endl;
			for (auto it = inputs.begin(); it != inputs.end(); it++)
				cerr << "\t" << *it << "\n";
		}; // else doHelp("Need to specify input files.");


		if (!vm.count("output"))
			doHelp("Need to specify output file(s).");
		string output = vm["output"].as< string >();

		std::shared_ptr<registry::IOhandler> handle, exportHandle;
		std::shared_ptr<rtmath::registry::DBhandler> dHandler;

		/*
		("from-db", "Perform search on database and select files matching criteria.")
		("match-hash", po::value<vector<string> >(), "Match lower hashes")
		("match-flake-type", po::value<vector<string> >(), "Match flake types")
		("match-dipole-spacing", po::value<vector<float> >(), "Match typical dipole spacings")
		("match-parent-flake-hash", po::value<vector<string> >(), "Match flakes having a given parent hash")
		("match-parent-flake", "Select the parent flakes")
		*/
		vector<string> matchHashes, matchFlakeTypes, matchParentHashes;
		rtmath::config::intervals<float> iDipoleSpacing;
		rtmath::config::intervals<size_t> iDipoleNumbers;
		bool matchParentFlakes;

		if (vm.count("match-hash")) matchHashes = vm["match-hash"].as<vector<string> >();
		if (vm.count("match-flake-type")) matchFlakeTypes = vm["match-flake-type"].as<vector<string> >();
		if (vm.count("match-parent-flake-hash")) matchParentHashes = vm["match-parent-flake-hash"].as<vector<string> >();
		if (vm.count("match-dipole-spacing")) iDipoleSpacing.append(vm["match-dipole-spacing"].as<vector<string>>());
		if (vm.count("match-dipole-numbers")) iDipoleNumbers.append(vm["match-dipole-numbers"].as<vector<string>>());

		if (vm.count("match-parent-flake")) matchParentFlakes = true;

		using namespace rtmath::ddscat::shapefile;
		auto collection = shapefile::makeCollection();
		auto query = shapefile::makeQuery();
		query->hashLowers.insert(matchHashes.begin(), matchHashes.end());
		query->flakeTypes.insert(matchFlakeTypes.begin(), matchFlakeTypes.end());
		query->refHashLowers.insert(matchParentHashes.begin(), matchParentHashes.end());
		query->dipoleNumbers = iDipoleNumbers;
		query->dipoleSpacings = iDipoleSpacing;

		bool supplementDb = false;
		supplementDb = vm["use-db"].as<bool>();
		bool fromDb = false;
		fromDb = vm["from-db"].as<bool>();



		auto opts = registry::IO_options::generate();
		auto optsExport = registry::IO_options::generate();

		opts->filename(output);

		auto dbcollection = rtmath::ddscat::shapefile::shapefile::makeCollection();
		auto qExisting = rtmath::ddscat::shapefile::shapefile::makeQuery();

		// Load in all local shapefiles, then perform the matching query
		vector<string> vinputs;
		for (auto it = inputs.begin(); it != inputs.end(); it++)
		{
			cerr << "Processing " << *it << endl;
			path pi(*it);
			if (!exists(pi)) throw rtmath::debug::xMissingFile(it->c_str());
			if (is_directory(pi))
			{
				path ps = pi / "shape.dat";
				boost::shared_ptr<rtmath::ddscat::shapefile::shapefile> smain;

				if (exists(ps))
				{
					cerr << " found " << ps << endl;
					smain = rtmath::ddscat::shapefile::shapefile::generate(ps.string());
					collection->insert(smain);
				}
			}
			else {
				auto iopts = registry::IO_options::generate();
				iopts->filename(*it);
				try {
					vector<boost::shared_ptr<rtmath::ddscat::shapefile::shapefile> > shapes;
					// Handle not needed as the read context is used only once.
					if (rtmath::ddscat::shapefile::shapefile::canReadMulti(nullptr, iopts))
						rtmath::ddscat::shapefile::shapefile::readVector(nullptr, iopts, shapes, nullptr);
					else {
						boost::shared_ptr<rtmath::ddscat::shapefile::shapefile> s = rtmath::ddscat::shapefile::shapefile::generate(*it);
						shapes.push_back(s);
					}
					for (auto &s : shapes)
						collection->insert(s);
				}
				catch (std::exception &e)
				{
					cerr << e.what() << std::endl;
					continue;
				}
			}
		}

		// Perform the query, and then process the matched shapefiles
		auto res = query->doQuery(collection, fromDb, supplementDb, dHandler);

		ofstream out(output.c_str());

		/** Want to tabulate the following:
		- shape hash, effective ice radius (dipoles),

		For each (circum_sphere, ellipsoid, convex, voronoi):
		- aeff_v (dipoles), aeff_sa (dipoles), sa (dipoles), v (dipoles), f
		- object sa/v, sphere-normalized sa/v

		For each (sfc depth 0 to 6)
		- Number lattice sites available, # sites occupied, volume fraction
		**/
		out << "Hash\tAeff_ice_dipoles\t";
		out << "Aeff_V_di_Circum\tAeff_SA_di_Circum\tSA_di_Circum\tV_di_Circum\tf_Circum\tSA_V_di_Circum\tSA_V_norm_Circum\t"
			<< "Aeff_V_di_EllMax\tAeff_SA_di_EllMax\tSA_di_EllMax\tV_di_EllMax\tf_EllMax\tSA_V_di_EllMax\tSA_V_norm_EllMax\t"
			<< "Aeff_V_di_Convex\tAeff_SA_di_Convex\tSA_di_Convex\tV_di_Convex\tf_Convex\tSA_V_di_Convex\tSA_V_norm_Convex\t"
			<< "Aeff_V_di_Voronoi\tAeff_SA_di_Voronoi\tSA_di_Voronoi\tV_di_Voronoi\tf_Voronoi\tSA_V_di_Voronoi\tSA_V_norm_Voronoi\t";

		for (int i = 0; i <= 6; ++i)
			out << "# lattice available (" << i << ")\t# lattice occupied (" << i << ")\tVolume Fraction (" << i << ")\t";

		out << std::endl;

		auto processShape = [&](boost::shared_ptr < rtmath::ddscat::shapefile::shapefile > shp)
		{
			using rtmath::ddscat::shapefile::shapefile;
			using rtmath::ddscat::stats::shapeFileStats;
			//boost::shared_ptr<shapeFileStats> stats;
			// Load the shape file

			using namespace rtmath::Voronoi;
			cerr << "  Shape " << shp->hash().lower << endl;
			shp->loadHashLocal(); // Load the shape fully, if it was imported from a database

			//boost::shared_ptr<shapefile> shp = shapefile::generate(input);
			auto vd = shp->generateVoronoi("standard", VoronoiDiagram::generateStandard);

			auto depthSfc = vd->calcSurfaceDepth();
			auto cellmap = vd->getCellMap();

			auto stats = rtmath::ddscat::stats::shapeFileStats::genStats(shp);
			
			const double pi = boost::math::constants::pi<double>();
			//double lambda = units::conv_spec("GHz", "um").convert(ddOut->freq);
			//double sizep = 2. * pi * ddOut->aeff / lambda;
			//double Vice_um = pow(ddOut->aeff, 3.) * 4. * pi / 3;
			//double aeff_di = ddOut->aeff / data(ddOutput::stat_entries::D);
			double aeff_di = stats->aeff_dipoles_const;
			double Vice_di = pow(aeff_di, 3.) * 4. * pi / 3;
			//double SAice_um = 4. * pi * pow(ddOut->aeff, 2.);
			double SAice_di = 4. * pi * pow(aeff_di, 2.);
			double SA_V_ice_di = SAice_di / Vice_di;
			//double SA_V_ice_um = SAice_um / Vice_um;

			out << shp->hash().lower << "\t" << stats->aeff_dipoles_const << "\t";

			auto writeS = [&](const rtmath::ddscat::stats::shapeFileStatsBase::volumetric &obj)
			{
				out << obj.aeff_V << "\t" << obj.aeff_SA << "\t" << obj.SA 
					<< "\t" << obj.V << "\t" << obj.f << "\t";
				double SA_V_obj = obj.SA / obj.V;
				double SA_V_norm = SA_V_obj / SA_V_ice_di;
				out << SA_V_obj << "\t" << SA_V_norm << "\t";
			};
			writeS(stats->Scircum_sphere);
			writeS(stats->Sellipsoid_max);
			writeS(stats->Sconvex_hull);
			writeS(stats->SVoronoi_hull);

			for (int threshold = 0; threshold <= 6; ++threshold)
			{
				// Preserve the dipoles that make up the 'surface'
				size_t outerCount = 0; // Keeps track of the number of points on the 'outside'.

				// Keep track of the inner volume fraction
				size_t innerPointsTotal = 0;
				size_t innerPointsFilled = 0;

				// Also track all potential dipole sites
				// First, iterate over all of the entries in depthSfc to extract the constant surface points.
				for (int i = 0; i < depthSfc->rows(); ++i)
				{
					// First three arguments are the coordinate. Fourth is the surface depth.
					auto crds = depthSfc->block<1, 3>(i, 0);
					int depth = (int)(*depthSfc)(i, 3);
					if (depth < threshold)
						outerCount++;
					else innerPointsFilled++;
				}

				// Iterate over all entries in the cell map, while consulting the surface depth field.
				for (int i = 0; i < cellmap->rows(); ++i)
				{
					// i is the probe point id
					// The first three columns are the probe point coordinates
					// The fourth column is the the voronoi point id (row)
					auto crds = cellmap->block<1, 3>(i, 0);
					int voroid = (*cellmap)(i, 3);
					int depth = (int)(*depthSfc)(voroid, 3);
					if (depth >= threshold)
						innerPointsTotal++;
				}

				float f = (float)innerPointsFilled / (float)innerPointsTotal;
				if (!innerPointsTotal) f = -1.f;

				out << innerPointsTotal << "\t" << innerPointsFilled << "\t" << f << "\t";
			}

			out << std::endl;
		};


		for (const auto &s : *(res.first))
			processShape(rtmath::ddscat::shapefile::shapefile::generate(s));


	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}
