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
#include <Ryan_Serialization/serialization.h>
#pragma warning( pop ) 

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
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
			("inputvoro,v", po::value<vector<string> >(), "Input voronoi diagrams")
			("output,o", po::value<string>(), "Output filename of Voronoi diagram file")
			("export-type", po::value<string>(), "Identifier to export (i.e. ar_rot_data)")
			("export,e", po::value<string>(), "Export filename (all shapes are combined into this)")
			("hash-voronoi", "Store standard Voronoi diagram")
			("store-shape", "Store standard shape data in output")
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

		vector<string> inputvoro;
		if (vm.count("inputvoro"))
		{
			inputvoro = vm["inputvoro"].as<vector<string> >();
			cerr << "Input voronoi diagrams are:" << endl;
			for (auto it = inputvoro.begin(); it != inputvoro.end(); ++it)
				cerr << "\t" << *it << "\n";
		};

		// No vectors! They are hard to detect before readins, leading to input stream errors.
		bool doExport = false;
		std::string exportType, exportFilename;
		//if (vm.count("separate-outputs")) sepOutputs = true;

		bool storeShape = false;
		if (vm.count("store-shape")) storeShape = true;

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
			} else vinputs.push_back(*it);
		}

		vector<string> vdinputs;
		for (auto it = inputvoro.begin(); it != inputvoro.end(); ++it)
		{
			path pi(*it);
			if (!exists(pi)) throw rtmath::debug::xMissingFile(it->c_str());
			if (is_directory(pi)) continue;
			vdinputs.push_back(*it);
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
				if (shapefile::shapefile::canReadMulti(nullptr,iopts))
					shapefile::shapefile::readVector(nullptr, iopts, shapes);
				else {
					boost::shared_ptr<shapefile::shapefile> s(new shapefile::shapefile);
					s->readFile(*it);
					shapes.push_back(s);
				}
			} catch (std::exception &e)
			{
				cerr << e.what() << std::endl;
				continue;
			}
		}


		using namespace rtmath::Voronoi;
		vector<boost::shared_ptr<Voronoi::VoronoiDiagram> > voros;
		for (const auto &shp : shapes)
		{
			cerr << "Generating Voronoi diagrams for hash " << shp->hash().lower << endl;
			boost::shared_ptr<VoronoiDiagram> vd;
			vd = shp->generateVoronoi(
				std::string("standard"), VoronoiDiagram::generateStandard);
			vd->calcSurfaceDepth();
			vd->calcCandidateConvexHullPoints();

			voros.push_back(vd);
		}
		for (auto it = vdinputs.begin(); it != vdinputs.end(); ++it)
		{
			cerr << "Processing stored Voronoi data " << *it << endl;
			auto iopts = registry::IO_options::generate(registry::IOhandler::IOtype::READONLY);
			iopts->filename(*it);
			try {
				// Handle not needed as the read context is used only once.
				if (VoronoiDiagram::canReadMulti(nullptr, iopts))
					VoronoiDiagram::readVector(nullptr, iopts, voros);
				else {
					boost::shared_ptr<VoronoiDiagram> s(new VoronoiDiagram);
					s->readFile(*it);
					voros.push_back(s);
				}
			}
			catch (std::exception &e)
			{
				cerr << e.what() << std::endl;
				continue;
			}
		}

		for (const auto &vd : voros)
		{
			cerr << "Processing hash " << vd->hash().lower << endl;

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

			//Stats.push_back(std::move(sstats));
		}

	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		cerr << endl;
		return 1;
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

