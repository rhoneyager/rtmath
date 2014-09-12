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
#include "../../rtmath/rtmath/refract.h"
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
			("input,i", po::value<vector<string> >(), "Input shape file(s)")
			("output,o", po::value<string >(), "Output file")

			//("output-dielectric", po::value<string>()->default_value("dielInterior.tab"),
			//"Output effective dielectric file for interior "
			//"(if interior uniformization is used)")
			//("frequency,f", po::value<double>(), "Frequency (GHz) if using an effective medium")
			//("temperature,T", po::value<double>(), "Temperature (K) if using an effective medium")
			//("diel-method", po::value<string>()->default_value("Maxwell-Garnett-Ellipsoids"), "Method used to calculate the resulting dielectric "
			//"(Sihvola, Debye, Maxwell-Garnett-Spheres, Maxwell-Garnett-Ellipsoids). "
			//"Only matters if volume fractions are given. Then, default is Sihvola.")

			("dipole-spacing,d", po::value<double>(), "Set dipole spacing for file exports.")
			("use-db", po::value<bool>()->default_value(true), "Use database to supplement loaded information")
			("from-db", po::value<bool>()->default_value(false), "Perform search on database and select files matching criteria.")
			("match-hash", po::value<vector<string> >()->multitoken(), "Match lower hashes")
			("match-flake-type", po::value<vector<string> >()->multitoken(), "Match flake types")
			("match-dipole-spacing", po::value<vector<float> >()->multitoken(), "Match typical dipole spacings")
			("match-dipole-numbers", po::value<vector<size_t> >()->multitoken(), "Match typical dipole numbers")
			("match-parent-flake-hash", po::value<vector<string> >()->multitoken(), "Match flakes having a given parent hash")
			("match-parent-flake", "Select the parent flakes")

			("use-effective-dielectric", "If specified, then all interior points will be replaced by "
			"an effective dielectric (scaled based on filled vs. total volume fraction. ")
			("threshold", po::value<int>()->default_value(3), "All dipole sites greater than or equal to "
			"this threshold are subject to randomization.")
			
			//("update-db", "Insert shape file entries into database")
			("tag", po::value<vector<string> >()->multitoken(), "Using \"key=value pairs\", add tags to the output (hdf5; not with .shp files)")
			("flake-type", po::value<string>(), "Specify flake type (e.g. oblate, oblate_small, prolate, ...")
			//("list-hash", "Write the hashes of each processed shapefile to stdout (for scripting)")
			//("hash-shape", "Store shapefile hash")
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

		int threshold = vm["threshold"].as<int>();
		bool useEffectiveDiel = false;
		if (vm.count("use-effective-dieelctric")) useEffectiveDiel = true;
		string outputDiel;
		if (vm.count("output-dielectric")) outputDiel = vm["output-dielectric"].as<string>();

		std::string sFlakeType;
		if (vm.count("flake-type")) sFlakeType = vm["flake-type"].as<string>();


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

			// Preserve the dipoles that make up the 'surface'
			boost::shared_ptr<Eigen::MatrixXi > resPts(new Eigen::MatrixXi());
			resPts->resize(shp->numPoints, 3);
			size_t outerCount = 0; // Keeps track of the number of points on the 'outside'.

			// Keep track of the inner volume fraction
			size_t innerPointsTotal = 0;
			size_t innerPointsFilled = 0;

			// Also track all potential dipole sites
			std::vector<Eigen::Matrix3i> potentialInnerPoints;
			potentialInnerPoints.reserve(cellmap->rows());

			// First, iterate over all of the entries in depthSfc to extract the constant surface points.
			for (int i = 0; i < depthSfc->rows(); ++i)
			{
				// First three arguments are the coordinate. Fourth is the surface depth.
				auto crds = depthSfc->block<1, 3>(i, 0);
				int depth = (int)(*depthSfc)(i, 3);
				if (depth < threshold)
				{
					resPts->block<1, 3>(outerCount, 0) = crds.cast<int>();
					outerCount++;
				}
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
				{
					innerPointsTotal++;
					Eigen::Matrix3i p;
					p.block<1, 3>(0, 0) = crds.block<1, 3>(0, 0);
					potentialInnerPoints.push_back(std::move(p));
				}
			}

			cout << "There are " << shp->numPoints << " points total.\n"
				<< "The threshold is " << threshold << ".\n"
				<< "The surface has " << outerCount << " points.\n"
				<< "The interior has " << innerPointsFilled << " filled points, "
				"and " << innerPointsTotal << " total potential sites.\n"
				<< "The inner fraction is " << 100.f * (float)innerPointsFilled
				/ (float)innerPointsTotal << " percent." << std::endl;

			bool doEffDiel = false;
			if (vm.count("use-effective-dielectric")) doEffDiel = true;

			boost::shared_ptr<Eigen::MatrixXi > resDiels(new Eigen::MatrixXi());
			resDiels->resize(resPts->rows(), 3);
			resDiels->setOnes();

			if (doEffDiel)
			{
				/*
				if (!vm.count("frequency")) doHelp("When using an effective dielectric, need to specify frequency and temperature");
				if (!vm.count("temperature")) doHelp("When using an effective dielectric, need to specify frequency and temperature");
				double freq = vm["frequency"].as<double>();
				double temp = vm["temperature"].as<double>();
				string method = vm["diel-method"].as<string>();

				complex<double> mAir(1.0, 0);
				complex<double> mIce, mEff;
				double fIce = innerPointsFilled / innerPointsTotal;
				rtmath::refract::mIce(freq, temp, mIce);
				if (method == "Sihvola")
				rtmath::refract::sihvola(mIce, mAir, fIce, 0.85, mEff);
				else if (method == "Debye")
				rtmath::refract::debyeDry(mIce, mAir, fIce, mEff);
				else if (method == "Maxwell-Garnett-Spheres")
				rtmath::refract::maxwellGarnettSpheres(mIce, mAir, fIce, mEff);
				else if (method == "Maxwell-Garnett-Ellipsoids")
				rtmath::refract::maxwellGarnettEllipsoids(mIce, mAir, fIce, mEff);
				else {
				cerr << "Unknown dielectric method: " << method << endl;
				throw rtmath::debug::xBadInput(method.c_str());
				}
				*/

				resPts->conservativeResize(potentialInnerPoints.size() + outerCount, 3);
				resDiels->conservativeResize(resPts->rows(), 3);
				//resDiels->setOnes();


				for (size_t i = 0; i < (size_t)potentialInnerPoints.size(); ++i)
				{
					resPts->block<1, 3>(outerCount + i, 0) = potentialInnerPoints[i].block<1, 3>(0, 0);
					resDiels->block<1, 3>(outerCount + i, 0).setConstant(2);
				}


			}
			else {
				// Scramble the potential inner points listing.
				//auto myrandom = [](int i) {return std::rand()%i;};
				std::srand(unsigned(std::time(0)));
				std::random_shuffle(potentialInnerPoints.begin(), potentialInnerPoints.end());
				// After shuffling, take the first innerPointsFilled and write them to the resultant shape.
				for (size_t i = 0; i < (size_t)innerPointsFilled; ++i)
				{
					resPts->block<1, 3>(outerCount + i, 0) = potentialInnerPoints[i].block<1, 3>(0, 0);
				}
			}

			// All points are now filled. Write a shape file from the data.
			// Not using the standard ddscat class at first, as it has to be read back in.
			std::ostringstream out;
			if (doEffDiel) 
				out << shp->desc << " - effective dielectric past threshold " << threshold << "\n";
			else 
				out << shp->desc << " - randomized past threshold " << threshold << "\n";
			out << resPts->rows() << std::endl;
			out << "1.0 0.0 0.0\n0.0 1.0 0.0\n1 1 1\n0 0 0\n";
			out << "No. ix iy iz dx dy dz\n";
			for (size_t i = 0; i < (size_t)resPts->rows(); ++i)
				out << i + 1 << "\t" << (*resPts)(i, 0) << "\t" << (*resPts)(i, 1) << "\t" <<
				(*resPts)(i, 2) << "\t" << (*resDiels)(i, 0) << "\t" <<
				(*resDiels)(i, 1) << "\t" << (*resDiels)(i, 2) << "\n";
			std::string ostr = out.str();
			std::istringstream in(ostr);
			auto oshp = shapefile::generate(in);
			oshp->fixStats();
			oshp->standardD = shp->standardD;
			if (vm.count("dipole-spacing"))
				oshp->standardD = (float) vm["dipole-spacing"].as<double>();
			oshp->tags.insert(std::pair<std::string, std::string>("decimation", "none"));
			{
				std::ostringstream pstr;
				pstr << "inner_";
				pstr << threshold;
				if (doEffDiel) pstr << "_eff";
				else pstr << "_rnd";

				oshp->tags.insert(std::pair<std::string, std::string>("perturbation", pstr.str()));
			}
			
			oshp->tags.insert(std::pair<std::string, std::string>("inner-perturbation-threshold", boost::lexical_cast<std::string>(threshold)));
			oshp->tags.insert(std::pair<std::string, std::string>("inner-perturbation-numSurfacePoints", boost::lexical_cast<std::string>(outerCount)));
			oshp->tags.insert(std::pair<std::string, std::string>("inner-perturbation-numInnerLatticeSites", boost::lexical_cast<std::string>(innerPointsTotal)));
			oshp->tags.insert(std::pair<std::string, std::string>("inner-perturbation-numOccupiedInnerLatticeSites", boost::lexical_cast<std::string>(innerPointsFilled)));
			if (sFlakeType.size())
				oshp->tags.insert(pair<string, string>("flake_classification", sFlakeType));
			oshp->tags.insert(std::pair<string, string>("flake_reference", shp->hash().string()));

			if (output.size())
				handle = oshp->writeMulti(handle, opts);

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
