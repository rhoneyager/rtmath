/* voronoi
 * Manipulates and generates Voronoi diagrams for later use.
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
#include <boost/serialization/vector.hpp>

#include <Ryan_Debug/debug.h>
//#include <Ryan_Serialization/serialization.h>
#pragma warning( pop ) 

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"


int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-voronoi\n\n";

		namespace po = boost::program_options;

		po::positional_options_description p;
		//p.add("inputshp", -1);

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		//Ryan_Serialization::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("inputshp,s", po::value< vector<string> >()->multitoken(), "Input shape files")
			("inputvoro,v", po::value<vector<string> >()->multitoken(), "Input voronoi diagrams")
			("inputrun,r", po::value<vector<string> >()->multitoken(), "Input ddscat runs")
			("output,o", po::value<string>(), "Output filename of Voronoi diagram file")
			("export-type", po::value<string>(), "Identifier to export (i.e. ar_rot_data)")
			("export,e", po::value<string>(), "Export filename (all shapes are combined into this)")
			("hash-voronoi", "Store standard Voronoi diagram")
			("store-shape", "Store standard shape data in output")

			("use-db", po::value<bool>()->default_value(true), "Use database to supplement loaded information")
			("from-db", po::value<bool>()->default_value(false), "Perform search on database and select files matching criteria.")
			("match-hash", po::value<vector<string> >()->multitoken(), "Match lower hashes")
			("match-flake-type", po::value<vector<string> >()->multitoken(), "Match flake types")
			("match-dipole-spacing", po::value<vector<std::string> >()->multitoken(), "Match typical dipole spacings")
			("match-dipole-numbers", po::value<vector<std::string> >()->multitoken(), "Match typical dipole numbers")
			("match-parent-flake-hash", po::value<vector<string> >()->multitoken(), "Match flakes having a given parent hash")
			//("match-parent-flake", "Select the parent flakes")
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
		//Ryan_Serialization::process_static_options(vm);

		vector<string> inputshp;
		if (vm.count("inputshp")) inputshp = vm["inputshp"].as< vector<string> >(); 
		vector<string> inputvoro;
		if (vm.count("inputvoro")) inputvoro = vm["inputvoro"].as<vector<string> >();
		vector<string> inputrun;
		if (vm.count("inputrun")) inputrun = vm["inputrun"].as<vector<string> >();
		string output;
		if (vm.count("output")) output = vm["output"].as<string>();
		bool storeShape = false;
		if (vm.count("store-shape")) storeShape = true;
		bool doExport = false;

		bool matchParentFlakes;
		vector<string> matchHashes, matchFlakeTypes, matchPols, matchRunUuids, matchParentHashes;
		if (vm.count("match-hash")) matchHashes = vm["match-hash"].as<vector<string> >();
		if (vm.count("match-flake-type")) matchFlakeTypes = vm["match-flake-type"].as<vector<string> >();
		rtmath::config::intervals<float> iDipoleSpacing;
		rtmath::config::intervals<size_t> iDipoleNumbers;
		if (vm.count("match-dipole-spacing")) iDipoleSpacing.append(vm["match-dipole-spacing"].as<vector<string>>());
		if (vm.count("match-dipole-numbers")) iDipoleNumbers.append(vm["match-dipole-numbers"].as<vector<string>>());
		if (vm.count("match-parent-flake")) matchParentFlakes = true;
		if (vm.count("match-parent-flake-hash")) matchParentHashes = vm["match-parent-flake-hash"].as<vector<string> >();

		using std::vector;
		using namespace rtmath::ddscat;

		auto collection = shapefile::shapefile::makeCollection();
		auto query = shapefile::shapefile::makeQuery();
		query->hashLowers.insert(matchHashes.begin(), matchHashes.end());
		query->flakeTypes.insert(matchFlakeTypes.begin(), matchFlakeTypes.end());
		query->refHashLowers.insert(matchParentHashes.begin(), matchParentHashes.end());
		query->dipoleNumbers = iDipoleNumbers;
		query->dipoleSpacings = iDipoleSpacing;

		bool supplementDb = false;
		supplementDb = vm["use-db"].as<bool>();
		bool fromDb = false;
		fromDb = vm["from-db"].as<bool>();


		auto dbcollection = shapefile::shapefile::makeCollection();
		auto qExisting = shapefile::shapefile::makeQuery();


		std::string exportType, exportFilename;
		/*
		{
			cerr << "Input shape files are:" << endl;
			for (auto it = inputshp.begin(); it != inputshp.end(); ++it)
				cerr << "\t" << *it << "\n";
			cerr << "Input voronoi diagrams are:" << endl;
			for (auto it = inputvoro.begin(); it != inputvoro.end(); ++it)
				cerr << "\t" << *it << "\n";
			cerr << "Input ddscat runs are:" << endl;
			for (auto it = inputrun.begin(); it != inputrun.end(); ++it)
				cerr << "\t" << *it << "\n";
			cerr << "Outputting to: " << output << endl;
		}
		*/
		if (vm.count("export"))
		{
			doExport = true;
			exportType = vm["export-type"].as<string>();
			exportFilename = vm["export"].as<string>();
			cerr << "Exporting to: " << exportFilename << endl;
		}

		// Setup for output
		std::shared_ptr<registry::IOhandler> handle, exportHandle;
		std::shared_ptr<rtmath::registry::DBhandler> dHandler;
		auto opts = registry::IO_options::generate();
		auto optsExport = registry::IO_options::generate();
		//opts->filetype(ctype);
		opts->exportType(exportType);
		opts->filename(output);
		optsExport->filename(exportFilename);
		optsExport->exportType(exportType);
		//opts->setVal("key", sstats._shp->filename);
		//optsExport->setVal("key", sstats._shp->filename);

		std::set<rtmath::HASH_t> already_done; // Prevent duplicate runs

		using namespace rtmath::ddscat;
		using namespace rtmath::Voronoi;
		auto doProcess = [&](
			boost::shared_ptr<const Voronoi::VoronoiDiagram> vd
			)
		{
			cerr << "Processing hash " << vd->hash().string() << endl;
			already_done.insert(vd->hash());
			
			if (vm.count("hash-voronoi"))
				vd->writeToHash();

			if (output.size())
				handle = vd->writeMulti(handle, opts);
			//	handle = sstats.writeMulti(handle, opts);
			if (doExport)
				exportHandle = vd->writeMulti(exportHandle, optsExport);
			//	exportHandle = sstats.writeMulti(exportHandle, optsExport);

			if (storeShape)
			{
				/// \todo Add an in-memory cache to hash.cpp and hash.h
				auto shp = shapefile::shapefile::loadHash(vd->hash().string());
				if (output.size())
					handle = shp->writeMulti(handle, opts);
			}
		};


		for (auto it = inputvoro.begin(); it != inputvoro.end(); ++it)
		{
			path pi(*it);
			if (!exists(pi)) RTthrow(rtmath::debug::xMissingFile())
				<< rtmath::debug::file_name(*it);
			if (is_directory(pi)) continue;
			
			cerr << "Processing stored Voronoi data " << *it << endl;
			auto iopts = registry::IO_options::generate(registry::IOhandler::IOtype::READONLY);
			iopts->filename(*it);
			try {
				vector<boost::shared_ptr<Voronoi::VoronoiDiagram> > voros;
				// Handle not needed as the read context is used only once.
				if (VoronoiDiagram::canReadMulti(nullptr, iopts))
					VoronoiDiagram::readVector(nullptr, iopts, voros, nullptr);
				else {
					boost::shared_ptr<VoronoiDiagram> s(new VoronoiDiagram);
					s->readFile(*it);
					voros.push_back(s);
				}

				for (auto &vd : voros)
				{
					if (!already_done.count(vd->hash()))
						doProcess(std::move(vd)); // Invalidate initial pointer to remove from memory after processing
				}
			} catch (std::exception &e) {
				cerr << e.what() << std::endl;
				continue;
			}
		}

		for (auto it = inputshp.begin(); it != inputshp.end(); ++it)
		{
			path ps;
			path pi(*it);
			if (!exists(pi)) { cerr << "Misssing file " << pi << endl; continue; }
			if (is_directory(pi))
			{
				ps = pi / "shape.dat";
				if (!exists(ps)) { cerr << "Misssing file " << ps << endl; continue; }
				//else throw rtmath::debug::xPathExistsWrongType(it->c_str());
			}
			else ps = pi;

			try {
				auto iopts = registry::IO_options::generate(registry::IOhandler::IOtype::READONLY);
				iopts->filename(*it);
				// Handle not needed as the read context is used only once.
				if (shapefile::shapefile::canReadMulti(nullptr, iopts)) {
					vector<boost::shared_ptr<shapefile::shapefile> > shapes;
					shapefile::shapefile::readVector(nullptr, iopts, shapes, query);
					for (auto &s : shapes)
						collection->insert(s);
				}
				else {
					boost::shared_ptr<shapefile::shapefile> s = shapefile::shapefile::generate(*it);
					collection->insert(s);
				}
			}
			catch (std::exception &e) {
				cerr << e.what() << std::endl;
				continue;
			}
		}
		// Perform the query, and then process the matched shapefiles
		auto res = query->doQuery(collection, fromDb, supplementDb, dHandler);


		for (const auto &s : *(res.first))
		{
			auto shp = rtmath::ddscat::shapefile::shapefile::generate(s);
			if (already_done.count(shp->hash())) continue;
			shp->loadHashLocal(); // Load the shape fully, if it was imported from a database

			cerr << "Generating Voronoi diagrams for hash " << shp->hash().lower << endl;
			boost::shared_ptr<VoronoiDiagram> vd;
			vd = shp->generateVoronoi(
				std::string("standard"), VoronoiDiagram::generateStandard);
			vd->calcSurfaceDepth();
			vd->calcCandidateConvexHullPoints();

			doProcess(std::move(vd));
		}

		for (auto it = inputrun.begin(); it != inputrun.end(); ++it)
		{
			path ps;
			path pi(*it);
			if (!exists(pi)) { cerr << "Misssing file " << pi << endl; continue; }
			ps = pi;
			vector<boost::shared_ptr<ddscat::ddOutput> > runs;

			try {
				auto iopts = registry::IO_options::generate(registry::IOhandler::IOtype::READONLY);
				iopts->filename(*it);
				// Handle not needed as the read context is used only once.
				if (is_directory(ps))
				{
					// Input is a ddscat run
					boost::shared_ptr<ddOutput> s(new ddOutput);
					s = ddOutput::generate(ps.string());
					runs.push_back(s);
				}
				else if (ddOutput::canReadMulti(nullptr, iopts))
					ddOutput::readVector(nullptr, iopts, runs, nullptr);
				else {
					// This fallback shouldn't happen...
					boost::shared_ptr<ddOutput> s(new ddOutput);
					s->readFile(*it);
					runs.push_back(s);
				}
			}
			catch (std::exception &e) {
				cerr << e.what() << std::endl;
				continue;
			}

			for (const auto &r : runs)
			{
				cerr << "Generating Voronoi diagrams for hash " << r->shapeHash.lower << endl;
				auto shp = shapefile::shapefile::loadHash(r->shapeHash);
				if (shp) {
					if (already_done.count(shp->hash())) continue;
					boost::shared_ptr<VoronoiDiagram> vd;
					vd = shp->generateVoronoi(
						std::string("standard"), VoronoiDiagram::generateStandard);
					vd->calcSurfaceDepth();
					vd->calcCandidateConvexHullPoints();

					doProcess(std::move(vd));
				}
				else std::cerr << "Shape file matching hash " 
					<< r->shapeHash.string() << " was not found." << std::endl;
			}
		}




	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

