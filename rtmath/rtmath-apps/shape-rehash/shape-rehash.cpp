/* shape-rehash
 * This program is designed for updating hashed shapestats when the stats calculation algorithms change.
 * It ensures that the most recent stats always exist for a shape. If stats are not found, it recreates 
 * the stats file from defaults.
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
		cerr << "rtmath-shape-rehash\n\n";

		namespace po = boost::program_options;

		po::positional_options_description p;

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		rtmath::debug::add_options(cmdline, config, hidden);
		ddscat::shapeFileStats::add_options(cmdline, config, hidden);
		Ryan_Serialization::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			;

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);
		Ryan_Serialization::process_static_options(vm);
		ddscat::shapeFileStats::process_static_options(vm);

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << "\n";
			if (message.size())
				cerr << message << "\n";
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");
		

		boost::filesystem::path pShapeDir, pStatsDir;
		ddscat::shapeFileStats::getHashPaths(pShapeDir, pStatsDir);

		// Iterate through the hashed shapefile list and get the hashes
		vector<string> hashes;
		{
			using namespace boost::filesystem;
			if (!exists(pShapeDir)) throw rtmath::debug::xMissingFile(pShapeDir.string().c_str());
			vector<path> cands;
			copy(recursive_directory_iterator(pShapeDir,symlink_option::recurse), 
				recursive_directory_iterator(), back_inserter(cands));
			for (const auto &f : cands)
			{
				if (is_directory(f)) continue;
				std::string hash, meth;
				Ryan_Serialization::uncompressed_name(f.string(), hash, meth);
				hashes.push_back(hash);
			}
		}

		for (const string &hash : hashes)
		{
			// Load the shape file
			cerr << "Processing " << hash << endl;
			path pShapeHashed = findHash(pShapeDir, hash);

			rtmath::ddscat::shapefile::shapefile shp(pShapeHashed.string());
			//HASH_t hash = shp.hash();

			path pStatsHashed = storeHash(pStatsDir,hash);
			rtmath::ddscat::shapeFileStats sstats;
			if (Ryan_Serialization::detect_compressed(pStatsHashed.string()))
			{
				std::cerr << "\tStats file with hash already exists. Appending.\n";
				try {
					Ryan_Serialization::read<rtmath::ddscat::shapeFileStats>
						(sstats, pStatsHashed.string(), "rtmath::ddscat::shapeFileStats");
				} catch (std::exception &e)
				{
					std::cerr << "\tStats file read error occurred. Skipping file. Reason:\n";
					cerr << "\t" << e.what() << endl;
					continue;
				}
				if (sstats.needsUpgrade())
				{
					cerr << "\tUpgrade needed.\n";
					sstats.upgrade();
				} else cerr << "\tNo upgrade needed.\n";
			} else {
				cerr << "\tCalculating statistics.\n";
				sstats = rtmath::ddscat::shapeFileStats(shp);
			}
			Ryan_Serialization::write<rtmath::ddscat::shapeFileStats>(sstats, 
				pStatsHashed.string(), "rtmath::ddscat::shapeFileStats", true);
		}

		cerr << "Done." << endl;
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

