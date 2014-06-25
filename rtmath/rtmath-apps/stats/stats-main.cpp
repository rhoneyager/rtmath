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
			("input-stats,i", po::value<vector<string> >(), "Input stats data")
			("input-run,r", po::value<vector<string> >(), "Input stored run (used to get shape orientations)")
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
			);

		vector<string> inputshp;
		if (vm.count("input-shp"))
		{
			inputshp = vm["input-shp"].as< vector<string> >();
			cerr << "Input shape files are:" << endl;
			for (auto it = inputshp.begin(); it != inputshp.end(); ++it)
				cerr << "\t" << *it << "\n";
		};

		vector<string> inputstats;
		if (vm.count("input-stats"))
		{
			inputstats = vm["input-stats"].as<vector<string> >();
			cerr << "Input stats files are:" << endl;
			for (auto it = inputstats.begin(); it != inputstats.end(); ++it)
				cerr << "\t" << *it << "\n";
		};

		vector<string> inputrun;
		if (vm.count("input-run"))
		{
			inputrun = vm["input-run"].as<vector<string> >();
			cerr << "Input DDSCAT run files are:" << endl;
			for (auto it = inputrun.begin(); it != inputrun.end(); ++it)
				cerr << "\t" << *it << "\n";
		};

		// No vectors! They are hard to detect before readins, leading to input stream errors.
		bool doExport = false;
		std::string exportType, exportFilename;
		//if (vm.count("separate-outputs")) sepOutputs = true;

		string output;

		if (vm.count("output"))
		{
			output = vm["output"].as<string>();
			cerr << "Outputting to: " << output << endl;
		}

		if (vm.count("export"))
		{
			doExport = true;
			exportType = vm["export-type"].as<string>();
			exportFilename = vm["export"].as<string>();
			cerr << "Exporting to: " << exportFilename << endl;
		}

		// Validate input files
		vector<string> vinputs;
		for (auto it = inputshp.begin(); it != inputshp.end(); ++it)
		{
			path pi(*it);
			if (!exists(pi)) throw rtmath::debug::xMissingFile(it->c_str());
			if (is_directory(pi))
			{
				path ps = pi / "shape.dat";
				if (exists(ps)) vinputs.push_back(ps.string());
				else continue;
				//else throw rtmath::debug::xPathExistsWrongType(it->c_str());
			}
			else vinputs.push_back(*it);
		}
		vector<string> sinputs;
		for (auto it = inputstats.begin(); it != inputstats.end(); ++it)
		{
			path pi(*it);
			if (!exists(pi)) throw rtmath::debug::xMissingFile(it->c_str());
			if (is_directory(pi)) continue;
			sinputs.push_back(*it);
		}

		// Storing the shape data as a shape + desired rotations
		using namespace ddscat;
		typedef std::pair<boost::shared_ptr<rotations>, boost::shared_ptr<stats::shapeFileStats> > rotpair;
		vector<rotpair> objs;


		for (auto it = inputrun.begin(); it != inputrun.end(); ++it)
		{
			cerr << "Processing " << *it << endl;
			vector<boost::shared_ptr<ddscat::ddOutput> > rinputs;
			path pi(*it);
			if (!exists(pi)) throw rtmath::debug::xMissingFile(it->c_str());
			if (is_directory(pi)) rinputs.push_back(ddscat::ddOutput::generate(*it, true));
			else {
				try {
					io::readObjs<ddscat::ddOutput>(rinputs, *it);
				} catch (std::exception &e) {
					cerr << e.what() << std::endl;
					continue;
				}
				
			}

			for (const auto &o : rinputs)
			{
				try {
					boost::shared_ptr<rotations> rotsrc = rotations::create(*(o->parfile));
					boost::shared_ptr<rotations> rot = rotations::create(
						rotsrc->bMin(), rotsrc->bMax(), rotsrc->bN(),
						rotsrc->tMin(), rotsrc->tMax(), rotsrc->tN(),
						0, 0, 1);
					o->loadShape();
					if (!o->stats)
						o->stats = stats::shapeFileStats::genStats(o->shape);
					objs.push_back(rotpair(rot, o->stats));
				} catch (rtmath::debug::xError &e) {
					cerr << "Cannot load results for hash " << o->shapeHash.string() << ".\n"
						<< e.what() << endl;
				}
			}
		}

		for (auto it = vinputs.begin(); it != vinputs.end(); ++it)
		{
			cerr << "Processing " << *it << endl;
			vector<boost::shared_ptr<ddscat::shapefile::shapefile> > sinputs;
			try {
				io::readObjs<ddscat::shapefile::shapefile>(sinputs, *it);
			} catch (std::exception &e) {
				cerr << e.what() << std::endl;
				continue;
			}
			for (const auto &o : sinputs)
				objs.push_back(rotpair(defaultrots, stats::shapeFileStats::genStats(o)));
		}


		for (auto it = sinputs.begin(); it != sinputs.end(); ++it)
		{
			cerr << "Processing stored stat data " << *it << endl;
			vector<boost::shared_ptr<stats::shapeFileStats> > sinputs;
			try {
				io::readObjs<stats::shapeFileStats>(sinputs, *it);
			}
			catch (std::exception &e) {
				cerr << e.what() << std::endl;
				continue;
			}
			for (const auto &o : sinputs)
				objs.push_back(rotpair(defaultrots, o));
		}




		std::shared_ptr<registry::IOhandler> handle, exportHandle;

		auto opts = registry::IO_options::generate();
		auto optsExport = registry::IO_options::generate();

		opts->exportType(exportType);
		opts->filename(output);
		optsExport->filename(exportFilename);
		optsExport->exportType(exportType);

		for (const auto &vd : objs)
		{
			cerr << "Processing hash " << vd.second->_shp->hash().lower << endl;

			std::map<boost::tuple<double, double, double>, size_t > rots;
			vd.first->getRots(rots);
			for (auto &r : rots)
			{
				cerr << " " << r.first.get<0>() << ", " << r.first.get<1>() << ", " << r.first.get<2>() << std::endl;
				vd.second->calcStatsRot(r.first.get<0>(), r.first.get<1>(), r.first.get<2>());
			}

			if (vm.count("hash-stats"))
				vd.second->writeToHash();

			if (output.size())
				handle = vd.second->writeMulti(handle, opts);
			//	handle = sstats.writeMulti(handle, opts);
			if (doExport)
				exportHandle = vd.second->writeMulti(exportHandle, optsExport);
			//	exportHandle = sstats.writeMulti(exportHandle, optsExport);

			//Stats.push_back(std::move(sstats));
		}

	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		cerr << endl;
		return 1;
	}
	catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

