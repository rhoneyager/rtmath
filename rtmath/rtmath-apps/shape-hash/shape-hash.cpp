/* shape-hash
 * This program is designed for loacating shape files and creating the appropriate links in the common hash 
 * directory (specified in rtmath configuration). If on the same filesystem, creates a hard link. Otherwise, 
 * creates a copy of the file wit the hash name as default. Writes compressed shapefile output and 
 * optionally writes stats output to the saves stats directory.
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
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/error/debug.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-shape-hash\n\n";

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		rtmath::debug::add_options(cmdline, config, hidden);
		ddscat::stats::shapeFileStats::add_options(cmdline, config, hidden);
		Ryan_Serialization::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value< vector<string> >(), "input shape files")
			("do-shapes", po::value<bool>()->default_value(true), "Create shape hash links")
			("do-stats", po::value<bool>()->default_value(true), "Create shape stats")
			;

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << "\n";
			if (message.size())
				cerr << message << "\n";
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");
		
		rtmath::debug::process_static_options(vm);
		Ryan_Serialization::process_static_options(vm);
		ddscat::stats::shapeFileStats::process_static_options(vm);

		bool doShapes = vm["do-shapes"].as<bool>();
		bool doStats = vm["do-stats"].as<bool>();

		boost::filesystem::path pShapeDir, pStatsDir;
		ddscat::stats::shapeFileStats::getHashPaths(pShapeDir, pStatsDir);

		vector<string> inputs = vm["input"].as< vector<string> >();
		if (vm.count("input"))
		{
			cerr << "Input files are:" << endl;
			for (auto it = inputs.begin(); it != inputs.end(); it++)
				cerr << "\t" << *it << "\n";
		} else {
			cerr << "Need to specify input files.\n" << desc << endl;
			return 1;
		}

		// Validate input files
		for (auto it = inputs.begin(); it != inputs.end(); it++)
		{
			path pi(*it);
			if (!exists(pi)) 
				throw rtmath::debug::xMissingFile(it->c_str());
			if (is_directory(pi)) 
				throw rtmath::debug::xPathExistsWrongType(it->c_str());
		}

		for (auto it = inputs.begin(); it != inputs.end(); it++)
		{
			// Load the shape file
			cerr << "Processing " << *it << endl;
			rtmath::ddscat::shapefile::shapefile shp(*it);
			HASH_t hash = shp.hash();
			path pHashName(boost::lexical_cast<std::string>(hash.lower));
			cerr << "\tHash is " << pHashName << endl;

			if (doShapes)
			{
				cerr << "\tCreating hashed shape file.\n";
				/// \todo See if hard-linking is possible
				// For now, just do a file write
				path pShapeHashed = storeHash(pShapeDir, hash);
				if (!Ryan_Serialization::detect_compressed(pShapeHashed.string()))
					shp.write(pShapeHashed.string());
				else cerr << "\tShape file hash already exists.\n";
			}
			if (doStats)
			{
				path pStatsHashedCand = storeHash(pStatsDir,hash);
				path pStatsHashed;
				Ryan_Serialization::serialization_method sm = Ryan_Serialization::select_format(pStatsHashedCand, pStatsHashed);

				rtmath::ddscat::stats::shapeFileStats sstats;
				if (Ryan_Serialization::detect_compressed(pStatsHashed.string()))
				{
					std::cerr << "\tStats file with hash already exists. Appending.\n";
					Ryan_Serialization::read<rtmath::ddscat::stats::shapeFileStats>
						(sstats, pStatsHashed.string(), "rtmath::ddscat::shapeFileStats");
				} else {
					cerr << "\tCalculating baseline statistics.\n";
					sstats = rtmath::ddscat::stats::shapeFileStats(shp);
				}
				Ryan_Serialization::write<rtmath::ddscat::stats::shapeFileStats>(sstats, 
					pStatsHashed.string(), "rtmath::ddscat::shapeFileStats", true);
			}
		}

		cerr << "Done." << endl;
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

