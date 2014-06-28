/* stats
* Manipulates and generates stats for later use.
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
#include <Ryan_Serialization/serialization.h>
#pragma warning( pop ) 

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"


int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-stats\n\n";

		namespace po = boost::program_options;

		po::positional_options_description p;
		//p.add("inputshp", -1);

		po::options_description desc("Allowed options"), cmdline("Command-line options"),
			config("Config options"), hidden("Hidden options"), oall("all options");
		//Ryan_Serialization::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("input-shp,s", po::value< vector<string> >()->multitoken(), "Input shape files")
			("input-stats,i", po::value<vector<string> >()->multitoken(), "Input stats data")
			("input-run,r", po::value<vector<string> >()->multitoken(), "Input stored run (used to get shape orientations)")
			("output,o", po::value<string>(), "Output filename of stats file")
			("export-type", po::value<string>(), "Identifier to export (i.e. ar_rot_data)")
			("export,e", po::value<string>(), "Export filename (all shapes are combined into this)")
			("hash-stats", "Store stats in hash database")
			;

		rtmath::debug::add_options(cmdline, config, hidden);
		rtmath::ddscat::stats::shapeFileStats::add_options(cmdline, config, hidden);

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
		rtmath::ddscat::stats::shapeFileStats::process_static_options(vm);
		//Ryan_Serialization::process_static_options(vm);

		string sbetas = vm["betas"].as<string>();
		paramSet<double> betas(sbetas);
		string sthetas = vm["thetas"].as<string>();
		paramSet<double> thetas(sthetas);
		//string sphis = vm["phis"].as<string>();
		//paramSet<double> phis(sphis);
		boost::shared_ptr<ddscat::rotations> defaultrots = ddscat::rotations::create(
			*(betas.begin()), *(betas.rbegin()), betas.size(),
			*(thetas.begin()), *(thetas.rbegin()), thetas.size(),
			0, 0, 1
			); // This is already done in the stats base code...

		vector<string> inputshp;
		if (vm.count("input-shp")) inputshp = vm["input-shp"].as< vector<string> >();
		vector<string> inputstats;
		if (vm.count("input-stats")) inputstats = vm["input-stats"].as<vector<string> >();
		vector<string> inputrun;
		if (vm.count("input-run")) inputrun = vm["input-run"].as<vector<string> >();
		string output;
		if (vm.count("output")) output = vm["output"].as<string>();
		bool doExport = false;
		std::string exportType, exportFilename;
		{
			cerr << "Input shape files are:" << endl;
			for (auto it = inputshp.begin(); it != inputshp.end(); ++it)
				cerr << "\t" << *it << "\n";
			cerr << "Input stats files are:" << endl;
			for (auto it = inputstats.begin(); it != inputstats.end(); ++it)
				cerr << "\t" << *it << "\n";
			cerr << "Input DDSCAT run files are:" << endl;
			for (auto it = inputrun.begin(); it != inputrun.end(); ++it)
				cerr << "\t" << *it << "\n";
			cerr << "Outputting to: " << output << endl;
		}
		if (vm.count("export"))
		{
			doExport = true;
			exportType = vm["export-type"].as<string>();
			exportFilename = vm["export"].as<string>();
			cerr << "Exporting to: " << exportFilename << endl;
		}

		// Setup for output
		std::shared_ptr<registry::IOhandler> handle, exportHandle;
		auto opts = registry::IO_options::generate();
		auto optsExport = registry::IO_options::generate();
		opts->exportType(exportType);
		opts->filename(output);
		optsExport->filename(exportFilename);
		optsExport->exportType(exportType);

		using namespace rtmath::ddscat;
		//for (const auto &vd : objs)
		auto doProcess = [&](
			rtmath::HASH_t hash, // always set
			boost::shared_ptr<const shapefile::shapefile> shp, // pre-loaded shape (if any). else, load from hash.
			boost::shared_ptr<stats::shapeFileStats> stats, // pre-loaded stats (if any). else, load from hash.
			boost::shared_ptr<rotations> rots // forced (non-default) rotations (for ddoutput)
			)
		{
			cerr << "Processing hash " << hash.string() << endl;

			if (!shp) shp = shapefile::shapefile::loadHash(hash);
			if (!shp) { cerr << " Shapefile not found" << endl; return; }

			if (!stats) stats = stats::shapeFileStats::genStats(shp);
			if (!stats) { cerr << " Unable to generate stats" << endl; return; }

			if (!rots) rots = defaultrots;

			std::map<boost::tuple<double, double, double>, size_t > rotmap;
			// Rotation setting done above
			rots->getRots(rotmap);
			cerr << " There are " << rotmap.size() << " rotations that will be calculated" << endl;
			for (auto &r : rotmap)
			{
				stats->calcStatsRot(r.first.get<0>(), r.first.get<1>(), r.first.get<2>());
			}

			if (vm.count("hash-stats"))
				stats->writeToHash();

			if (output.size())
				handle = stats->writeMulti(handle, opts);
			if (doExport)
				exportHandle = stats->writeMulti(exportHandle, optsExport);

			// At this point, the stats can be discarded to save memory
		};


		// Do stat processing first, in case it covers the other cases (useful when hashing)
		for (auto it = inputstats.begin(); it != inputstats.end(); ++it)
		{
			cerr << "Processing stat file " << *it << endl;
			path pi(*it);
			if (!exists(pi)) {cerr << "Misssing file " << pi << endl; continue;}
			if (is_directory(pi)) continue;

			vector<boost::shared_ptr<stats::shapeFileStats> > sinputs;
			try {
				io::readObjs<stats::shapeFileStats>(sinputs, *it);
			} catch (std::exception &e) {
				cerr << e.what() << std::endl;
				continue;
			}
			cerr << " " << sinputs.size() << " stat hashes found." << endl;
			for (const auto &o : sinputs)
				doProcess(o->_shp->hash(), o->_shp, o, defaultrots);
		}
		for (auto it = inputshp.begin(); it != inputshp.end(); ++it)
		{
			cerr << "Processing shape file " << *it << endl;
			path ps;
			path pi(*it);
			if (!exists(pi)) { cerr << "Misssing file " << pi << endl; continue; }
			if (is_directory(pi))
			{
				ps = pi / "shape.dat";
				if (!exists(ps)) { cerr << "Misssing file " << ps << endl; continue; }
			}
			else ps = pi;
			vector<boost::shared_ptr<ddscat::shapefile::shapefile> > sinputs;
			try {
				io::readObjs<ddscat::shapefile::shapefile>(sinputs, ps.string());
			} catch (std::exception &e) {
				cerr << e.what() << std::endl;
				continue;
			}
			cerr << " " << sinputs.size() << " shape hashes found." << endl;
			for (const auto &o : sinputs)
				doProcess(o->hash(), o, nullptr, defaultrots);
		}
		for (auto it = inputrun.begin(); it != inputrun.end(); ++it)
		{
			cerr << "Processing ddOutput file " << *it << endl;
			vector<boost::shared_ptr<ddscat::ddOutput> > rinputs;
			path pi(*it);
			if (!exists(pi)) { cerr << "Misssing file " << pi << endl; continue; }
			if (is_directory(pi)) rinputs.push_back(ddscat::ddOutput::generate(*it, true));
			else {
				try {
					io::readObjs<ddscat::ddOutput>(rinputs, *it);
				} catch (std::exception &e) {
					cerr << e.what() << std::endl;
					continue;
				}
				
			}
			cerr << " " << rinputs.size() << " hashes found." << endl;
			for (const auto &o : rinputs)
			{
				cerr << " Extracting hash " << o->shapeHash.string() << endl;
				boost::shared_ptr<rotations> rotsrc = rotations::create(*(o->parfile));
				boost::shared_ptr<rotations> rot = rotations::create(
					rotsrc->bMin(), rotsrc->bMax(), rotsrc->bN(),
					rotsrc->tMin(), rotsrc->tMax(), rotsrc->tN(),
					0, 0, 1);
				doProcess(o->shapeHash, nullptr, nullptr, rot);
			}
		}

	} catch (rtmath::debug::xError &err) {
		err.Display();
		cerr << endl;
		return 1;
	} catch (std::exception &e) {
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

