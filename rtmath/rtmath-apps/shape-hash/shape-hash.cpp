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

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("input,i", po::value< vector<string> >(), "input shape files")
			("hash-shape-dir", po::value<string>(), "Override the hash shape directory")
			("hash-stats-dir", po::value<string>(), "Override the hash stats directory")
			("do-shapes", po::value<bool>()->default_value(true), "Create shape hash links")
			("do-stats", po::value<bool>()->default_value(true), "Create shape stats")

			("betas,b", po::value<string>()->default_value("0"), "Specify beta rotations for stats")
			("thetas,t", po::value<string>()->default_value("0"), "Specify theta rotations for stats")
			("phis,p", po::value<string>()->default_value("0"), "Specify phi rotations for stats")
			("disable-qhull", "Disable qhull calculations for the stats");

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);    

		if (vm.count("help") || argc == 1) {
			cerr << desc << "\n";
			return 1;
		}

		if (vm.count("disable-qhull"))
			rtmath::ddscat::shapeFileStats::doQhull(false);
		bool doShapes = vm["do-shapes"].as<bool>();
		bool doStats = vm["do-stats"].as<bool>();

		/// \todo Add config parsing of command-line options, and chage how config is stored
		auto conf = rtmath::config::loadRtconfRoot();
		string shapeDir;
		auto chash = conf->getChild("ddscat")->getChild("hash");
		chash->getVal<string>("shapeDir",shapeDir);
		if (vm.count("hash-shape-dir")) shapeDir = vm["hash-shape-dir"].as<string>();
		string statsDir;
		chash->getVal<string>("statsDir",statsDir);
		if (vm.count("hash-stats-dir")) statsDir = vm["hash-stats-dir"].as<string>();
		// Check directory existence
		path pShapeDir(shapeDir), pStatsDir(statsDir);
		auto validateDir = [&](path p) -> bool
		{
			while (is_symlink(p))
				p = boost::filesystem::absolute(read_symlink(p), p.parent_path());
			if (!exists(p)) return false;
			if (is_directory(p)) return true;
			return false;
		};
		std::cerr << "Using shape hash directory " << pShapeDir << std::endl;
		std::cerr << "Using stats hash directory " << pStatsDir << std::endl;
		if (!validateDir(pShapeDir)) throw debug::xMissingFile(shapeDir.c_str());
		if (!validateDir(pStatsDir)) throw debug::xMissingFile(statsDir.c_str());

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

		// Specify beta, theta, phi rotations
		string sbeta = vm["betas"].as<string>();
		string stheta = vm["thetas"].as<string>();
		string sphi = vm["phis"].as<string>();

		paramSet<double> betas(sbeta);
		paramSet<double> thetas(stheta);
		paramSet<double> phis(sphi);

		//vector<rtmath::ddscat::shapeFileStats> Stats;
		//Stats.reserve(inputs.size());

		for (auto it = inputs.begin(); it != inputs.end(); it++)
		{
			// Load the shape file
			cerr << "Processing " << *it << endl;
			rtmath::ddscat::shapefile shp(*it);
			HASH_t hash = shp.hash();
			path pHashName(boost::lexical_cast<std::string>(hash.lower));
			cerr << "\tHash is " << pHashName << endl;

			if (doShapes)
			{
				cerr << "\tCreating hashed shape file.\n";
				/// \todo See if hard-linking is possible
				// For now, just do a file write
				path pShapeHashed = pShapeDir / pHashName;
				std::string a,b;
				if (!Ryan_Serialization::detect_compressed(pShapeHashed.string(), 
					a, b))
					shp.write(pShapeHashed.string(), true);
				else cerr << "\tShape file hash already exists.\n";
			}
			if (doStats)
			{
				path pStatsHashed = pStatsDir / pHashName;
				std::string a, b;
				rtmath::ddscat::shapeFileStats sstats;
				if (Ryan_Serialization::detect_compressed(pStatsHashed.string(), a, b))
				{
					std::cerr << "\tStats file with hash already exists. Appending.\n";
					Ryan_Serialization::read<rtmath::ddscat::shapeFileStats>
						(sstats, pStatsHashed.string(), "rtmath::ddscat::shapeFileStats");
				} else {
					sstats = rtmath::ddscat::shapeFileStats(shp);
					cerr << "\tCalculating baseline statistics.\n";
					sstats.calcStatsBase();
				}
				for (auto beta = betas.begin(); beta != betas.end(); beta++)
				{
					for (auto theta = thetas.begin(); theta != thetas.end(); theta++)
					{
						for (auto phi = phis.begin(); phi != phis.end(); phi++)
						{
							cerr << "\tCalculating rotation (beta,theta,phi): ("
								<< *beta << ", " << *theta << ", " << *phi << ")" << endl;
							sstats.calcStatsRot(*beta,*theta,*phi);
						}
					}
				}
				Ryan_Serialization::write<rtmath::ddscat::shapeFileStats>(sstats, 
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

