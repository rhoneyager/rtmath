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
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
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
			("inputshp,s", po::value< vector<string> >()->multitoken(), "Input shape files")
			("inputstats,i", po::value<vector<string> >(), "Input stats data")
			("output,o", po::value<string>(), "Output filename of stats file")
			("export-type", po::value<string>(), "Identifier to export (i.e. ar_rot_data)")
			("export,e", po::value<string>(), "Export filename (all shapes are combined into this)")
			("hash-stats", "Store stats in hash database")
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
		if (vm.count("inputshp"))
		{
			inputshp = vm["inputshp"].as< vector<string> >();
			cerr << "Input shape files are:" << endl;
			for (auto it = inputshp.begin(); it != inputshp.end(); ++it)
				cerr << "\t" << *it << "\n";
		};

		vector<string> inputstats;
		if (vm.count("inputstats"))
		{
			inputstats = vm["inputstats"].as<vector<string> >();
			cerr << "Input stats files are:" << endl;
			for (auto it = inputstats.begin(); it != inputstats.end(); ++it)
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

		std::shared_ptr<registry::IOhandler> handle, exportHandle;

		auto opts = registry::IO_options::generate();
		auto optsExport = registry::IO_options::generate();

		//opts->filetype(ctype);
		opts->exportType(exportType);
		opts->filename(output);
		optsExport->filename(exportFilename);
		optsExport->exportType(exportType);
		//opts->setVal("key", sstats._shp->filename);
		//optsExport->setVal("key", sstats._shp->filename);

		using std::vector;
		using namespace rtmath::ddscat;
		vector<boost::shared_ptr<shapefile::shapefile> > shapes;
		for (auto it = vinputs.begin(); it != vinputs.end(); ++it)
		{
			cerr << "Processing " << *it << endl;
			auto iopts = registry::IO_options::generate(registry::IOhandler::IOtype::READONLY);
			iopts->filename(*it);
			try {
				// Handle not needed as the read context is used only once.
				if (shapefile::shapefile::canReadMulti(nullptr, iopts))
					shapefile::shapefile::readVector(nullptr, iopts, shapes);
				else {
					boost::shared_ptr<shapefile::shapefile> s(new shapefile::shapefile);
					s->readFile(*it);
					shapes.push_back(s);
				}
			}
			catch (std::exception &e)
			{
				cerr << e.what() << std::endl;
				continue;
			}
		}


		using namespace rtmath::ddscat::stats;
		vector<boost::shared_ptr<shapeFileStats> > stats;
		for (const auto &shp : shapes)
		{
			cerr << "Generating stats for hash " << shp->hash().lower << endl;
			boost::shared_ptr<shapeFileStats> vd;
			vd = shapeFileStats::genStats(shp);

			stats.push_back(vd);
		}
		for (auto it = sinputs.begin(); it != sinputs.end(); ++it)
		{
			cerr << "Processing stored stat data " << *it << endl;
			auto iopts = registry::IO_options::generate(registry::IOhandler::IOtype::READONLY);
			iopts->filename(*it);
			try {
				// Handle not needed as the read context is used only once.
				if (shapeFileStats::canReadMulti(nullptr, iopts))
					shapeFileStats::readVector(nullptr, iopts, stats);
				else {
					boost::shared_ptr<shapeFileStats> s(new shapeFileStats);
					s->readFile(*it);
					stats.push_back(s);
				}
			}
			catch (std::exception &e)
			{
				cerr << e.what() << std::endl;
				continue;
			}
		}

		for (const auto &vd : stats)
		{
			cerr << "Processing hash " << vd->_shp->hash().lower << endl;

			if (vm.count("hash-stats"))
				vd->writeToHash();

			if (output.size())
				handle = vd->writeMulti(handle, opts);
			//	handle = sstats.writeMulti(handle, opts);
			if (doExport)
				exportHandle = vd->writeMulti(exportHandle, optsExport);
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

