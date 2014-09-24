/* Program matches flakes to their parents (for older perturbations) based on their number of dipoles.
* Database search is used to select the parents, and the children are provided via the console.
* The child entries are written in hdf5 form, and may later be registered in the database.
*/

#include <boost/math/constants/constants.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <sstream>
#include <map>
#include <vector>
#include <cstdlib>
#include <Ryan_Debug/debug.h>
//#include <Ryan_Serialization/serialization.h>

#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/ddscat/ddOriData.h"
#include "../../rtmath/rtmath/ddscat/ddRunSet.h"
#include "../../rtmath/rtmath/ddscat/dielTabFile.h"
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

int main(int argc, char** argv)
{
	using namespace std;
	try {
		cerr << "rtmath-shape-match-parent\n\n";

		namespace po = boost::program_options;
		using std::endl;
		using std::cerr;
		using std::string;
		using std::vector;

		po::options_description desc("Allowed options"), cmdline("Command-line options"),
			config("Config options"), hidden("Hidden options"), oall("all options");

		rtmath::debug::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("shape,s", po::value<vector<string> >()->multitoken(), "Select child shape files")
			("parent-shape,p", po::value<vector<string> >()->multitoken(), "Select parent shape files")
			("output,o", po::value<string>(), "Output hdf5 file for modified child shape information")
			
			("dipole-spacing,d", po::value<double>(), "Override the dipole spacing")
			("use-db", po::value<bool>()->default_value(true), "Use database to supplement loaded information")
			("from-db", po::value<bool>()->default_value(false), "Perform search on database and select files matching criteria.")
			("match-hash", po::value<vector<string> >()->multitoken(), "Match lower hashes")
			("match-flake-type", po::value<vector<string> >()->multitoken(), "Match flake types")
			("match-dipole-spacing", po::value<vector<float> >()->multitoken(), "Match typical dipole spacings")
			("match-dipole-numbers", po::value<vector<size_t> >()->multitoken(), "Match typical dipole numbers")
			;

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << endl;
			cerr << message << endl;
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");

		std::map<std::string, rtmath::ddscat::dataset> maps;

		vector<string> ishapes, iparents; // May be expanded by os.
		string soutput;

		if (!vm.count("shape")) doHelp("Need to select child shape files");
		if (vm.count("shape"))
			ishapes = vm["shape"].as<vector<string> >();
		if (vm.count("parent-shape"))
			iparents = vm["parent-shape"].as<vector<string> >();

		if (!vm.count("output")) doHelp("Need to specify output filename");
		soutput = vm["output"].as<string>();

		double dSpacing = 0;
		if (vm.count("dipole-spacing"))
			dSpacing = vm["dipole-spacing"].as<double>();

		std::shared_ptr<rtmath::registry::IOhandler> handle, exportHandle;
		std::shared_ptr<rtmath::registry::DBhandler> dHandler;

		/*
		("from-db", "Perform search on database and select files matching criteria.")
		("match-hash", po::value<vector<string> >(), "Match lower hashes")
		("match-flake-type", po::value<vector<string> >(), "Match flake types")
		("match-dipole-spacing", po::value<vector<float> >(), "Match typical dipole spacings")
		*/
		vector<string> matchHashes, matchFlakeTypes, matchParentHashes;
		rtmath::config::intervals<float> iDipoleSpacing;
		rtmath::config::intervals<size_t> iDipoleNumbers;

		if (vm.count("match-hash")) matchHashes = vm["match-hash"].as<vector<string> >();
		if (vm.count("match-flake-type")) matchFlakeTypes = vm["match-flake-type"].as<vector<string> >();
		if (vm.count("match-dipole-spacing")) iDipoleSpacing.append(vm["match-dipole-spacing"].as<vector<string>>());
		if (vm.count("match-dipole-numbers")) iDipoleNumbers.append(vm["match-dipole-numbers"].as<vector<string>>());


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

		auto opts = rtmath::registry::IO_options::generate();
		opts->filename(soutput);
		opts->setVal<double>("dSpacing", dSpacing);

		using namespace boost::filesystem;
		using namespace rtmath::ddscat;

		std::set<boost::shared_ptr<const rtmath::ddscat::shapefile::shapefile> > parent_srcs;
		std::set<boost::shared_ptr<const rtmath::ddscat::shapefile::shapefile> > children;

		auto loadShps = [&](const std::vector<std::string> & files, std::set<boost::shared_ptr<const rtmath::ddscat::shapefile::shapefile> >*res)
		{
			for (auto it = files.begin(); it != files.end(); ++it)
			{
				cerr << "Processing " << *it << endl;
				path pi(*it);
				if (!exists(pi)) throw rtmath::debug::xMissingFile(it->c_str());
				if (is_directory(pi))
				{
					path pt = pi / "target.out";
					path ps = pi / "shape.dat";
					boost::shared_ptr<rtmath::ddscat::shapefile::shapefile> smain;

					if (exists(ps))
					{
						cerr << " found " << ps << endl;
						smain = rtmath::ddscat::shapefile::shapefile::generate(ps.string());
						res->insert(smain);
					}
				}
				else {
					auto iopts = rtmath::registry::IO_options::generate();
					iopts->filename(*it);
					try {
						vector<boost::shared_ptr<rtmath::ddscat::shapefile::shapefile> > shapes;
						// Handle not needed as the read context is used only once.
						if (rtmath::ddscat::shapefile::shapefile::canReadMulti(nullptr, iopts))
							rtmath::ddscat::shapefile::shapefile::readVector(nullptr, iopts, shapes, query);
						else {
							boost::shared_ptr<rtmath::ddscat::shapefile::shapefile> s = rtmath::ddscat::shapefile::shapefile::generate(*it);
							shapes.push_back(s);
						}
						for (auto &s : shapes)
							res->insert(s);
					}
					catch (std::exception &e)
					{
						cerr << e.what() << std::endl;
						continue;
					}
				}
			}
		};
		loadShps(iparents, &parent_srcs);
		loadShps(ishapes, &children);
		for (const auto &a : parent_srcs) collection->emplace(a); // The sets have different comparators.
		
		// Perform the query, and then process the matched shapefiles
		auto res = query->doQuery(collection, fromDb, supplementDb, dHandler);
		// No need to fully load the parent shapes. I just want 
		// the shape hash, the dipole spacing and the number of dipoles.


		auto processShape = [&](boost::shared_ptr<const rtmath::ddscat::shapefile::shapefile> s)
		{
			cerr << "  Child shape " << s->hash().lower << endl;
			boost::shared_ptr<rtmath::ddscat::shapefile::shapefile> shp 
				= boost::const_pointer_cast<rtmath::ddscat::shapefile::shapefile>(s);
			shp->loadHashLocal(); // Load the shape fully, if it was imported from a database

			// Do search for parent
			auto parQ = rtmath::ddscat::shapefile::shapefile::makeQuery();
			parQ->dipoleNumbers.ranges.push_back(std::pair<size_t,size_t>(shp->numPoints, shp->numPoints));

			auto parC = parQ->doQuery(res.first, false, false, nullptr);

			if (!parC.first->size())
			{
				cerr << "\t\tCannot find parent match." << endl;
				return;
			}

			auto par = *(parC.first->begin());

			if (!shp->standardD)
			{
				if (par->standardD) shp->standardD = par->standardD;
				else if (dSpacing) shp->standardD = (float)dSpacing;
				else doHelp("Need to set standard dipole spacings for some of these files.");
			}

			// Get effective radius
			//double aeff_di = pow((float)shp->numPoints*3.f / (4.f*boost::math::constants::pi<float>()), 1.f / 3.f);
			//double aeff_um = aeff_di * shp->standardD;

			// Add parent hash information (flake_reference)
			shp->tags.emplace(std::pair<std::string, std::string>(
				"flake_reference", par->hash().string()));

			// Write output
			handle = shp->writeMulti(handle, opts);
		};

		for (const auto &s : children)
			processShape(s);


	}
	catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}
