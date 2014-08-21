/* shape
 * A program designed for analyzing ddscat shape.dat files and the associated ___ files output when 
 * ddscat runs. The shape file is loaded, and various properties of the shape are analyzed. 
 * In particular, the moments of inertia are calculated for the shape, and its most likely 
 * axis, given lamellar-like flow conditions, is calculated. This is important because it 
 * indicates if the shapes that are generated contain the appropriate angle calculations!
 * 
 * Also, ancillary useful quantities are calculated. Crystal mass is not, as these shapes may be scaled.
 * However, the number of dipoles is presented, along with crystal volume relations for a given 
 * radius. 
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
#pragma warning( pop ) 

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"



/// \todo Redo writeMultis to follow the new convention of using IO_options.
double dSpacing = 0;

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-shape\n\n";

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		ddscat::stats::shapeFileStats::add_options(cmdline, config, hidden);
		//Ryan_Serialization::add_options(cmdline, config, hidden);

		/// \todo Add unified read option, to pull shapefiles from other sources (useful for tagging)

		cmdline.add_options()
			("help,h", "produce help message")
			("dipole-spacing,d", po::value<double>(), "Set dipole spacing for file exports.")
			("input,i", po::value< vector<string> >(), "Input shape files")
			("from-db", "Perform search on database and select files matching criteria.")
			("match-hash", po::value<vector<string> >()->multitoken(), "Match lower hashes")
			("match-flake-type", po::value<vector<string> >()->multitoken(), "Match flake types")
			("match-dipole-spacing", po::value<vector<float> >()->multitoken(), "Match typical dipole spacings")
			("match-parent-flake-hash", po::value<vector<string> >()->multitoken(), "Match flakes having a given parent hash")
			("match-parent-flake", "Select the parent flakes")

			("output,o", po::value<string>(), "Output filename")
			("export-type", po::value<string>(), "Identifier to export (i.e. ar_rot_data)")
			("export,e", po::value<string>(), "Export filename (all shapes are combined into this)")
			("tag", po::value<vector<string> >(), "Using \"key=value pairs\", add tags to the output (not with .shp files)")
			("flake-type", po::value<string>(), "Specify flake type (e.g. oblate, oblate_small, prolate, ...")
			//("separate-outputs,s", "Vestigial option. Write separate output file for each input. Use default naming scheme.")
			("process-target-out", po::value<bool>()->default_value(0), "Process target.out files in addition to shape.dat files.")
			("update-db", "Insert shape file entries into database")
			("list-hash", "Write the hashes of each processed shapefile to stdout (for scripting)")
			("hash-shape", "Store shapefile hash")
			("hash-stats", "Store shapefile stats in hash location")
			("hash-voronoi", "Store standard Voronoi diagram")
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
		ddscat::stats::shapeFileStats::process_static_options(vm);

		vector<std::pair<string, string> > tags;
		vector<string> tags_pre;
		if (vm.count("tag"))
		{ // do tag splitting
			tags_pre = vm["tag"].as<vector<string> >();
			vector<string> out;
			for (const auto &v : tags_pre)
			{
				rtmath::config::splitVector(v, out, '=');
				if (out.size() < 2) out.push_back("");
				tags.push_back(std::pair<string, string>(out[0], out[1]));
				out.clear();
			}
		}

		if (vm.count("dipole-spacing"))
			dSpacing = vm["dipole-spacing"].as<double>();

		vector<string> inputs = vm["input"].as< vector<string> >();
		if (vm.count("input"))
		{
			cerr << "Input files are:" << endl;
			for (auto it = inputs.begin(); it != inputs.end(); it++)
				cerr << "\t" << *it << "\n";
		} else doHelp("Need to specify input files.");

		// sepOutputs is a vestigial option. I want everything in a separate file automatically. 
		// No vectors! They are hard to detect before readins, leading to input stream errors.
		bool sepOutputs = false;
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

		bool doTargetOut = vm["process-target-out"].as<bool>();


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
		vector<float> matchDipoleSpacings;
		bool matchParentFlakes;

		if (vm.count("match-hash")) matchHashes = vm["match-hash"].as<vector<string> >();
		if (vm.count("match-flake-type")) matchFlakeTypes = vm["match-flake-type"].as<vector<string> >();
		if (vm.count("match-parent-flake-hash")) matchParentHashes = vm["match-parent-flake-hash"].as<vector<string> >();
		if (vm.count("match-dipole-spacing")) matchDipoleSpacings = vm["match-dipole-spacing"].as<vector<float> >();
		if (vm.count("match-parent-flake")) matchParentFlakes = true;

		std::string sFlakeType;
		if (vm.count("flake-type")) sFlakeType = vm["flake-type"].as<string>();


		auto opts = registry::IO_options::generate();
		auto optsExport = registry::IO_options::generate();

		//opts->filetype(ctype);
		opts->exportType(exportType);
		opts->filename(output);
		opts->setVal<double>("dSpacing", dSpacing);
		optsExport->setVal<double>("dSpacing", dSpacing);
		optsExport->filename(exportFilename);
		optsExport->exportType(exportType);
		//opts->setVal("key", sstats._shp->filename);
		//optsExport->setVal("key", sstats._shp->filename);

		auto dbcollection = rtmath::ddscat::shapefile::shapefile::makeCollection();
		auto qExisting = rtmath::ddscat::shapefile::shapefile::makeQuery();

		// TODO: add a search first for existing files in the database, and then import the results.

		using namespace rtmath::ddscat;
		auto processShape = [&](boost::shared_ptr<shapefile::shapefile> shp)
		{
			cerr << "  Shape " << shp->hash().lower << endl;
			for (auto &t : tags)
				shp->tags.insert(t);

			if (vm.count("list-hash"))
				std::cout << shp->hash().lower << endl;
			if (sFlakeType.size())
				shp->tags.insert(pair<string, string>("flake_type", sFlakeType));
			if (dSpacing) shp->standardD = (float) dSpacing;
			if (vm.count("hash-shape")) shp->writeToHash();
			if (vm.count("hash-voronoi"))
			{
				using namespace rtmath::Voronoi;
				boost::shared_ptr<VoronoiDiagram> vd;
				// If the diagram is already hashed, generateVoronoi will just load the hashed copy
				vd = shp->generateVoronoi(
					std::string("standard"), VoronoiDiagram::generateStandard);
				// If already hashed, then this does nothing
				vd->writeToHash();
			}
			//if (vm.count("hash-stats")) stats->writeToHash();
			//cerr << "\tCalculating statistics" << endl;
			//rtmath::ddscat::stats::shapeFileStats sstats(shp);

			if (vm.count("update-db"))
			{
				dbcollection->insert(shp);

				if (dbcollection->size() > 500) // Do in batches. The final uneven batch is handled at the end of execution.
				{
					dHandler = shapefile::shapefile::updateCollection(dbcollection,
						shapefile::shapefile_db_registry::updateType::INSERT_ONLY, dHandler);
					dbcollection->clear();
				}
				//dHandler = im->updateEntry(rtmath::data::arm::arm_info_registry::updateType::INSERT_ONLY, dHandler);
			}

			if (output.size())
				handle = shp->writeMulti(handle, opts);
			//	handle = sstats.writeMulti(handle, opts);
			if (doExport)
				exportHandle = shp->writeMulti(exportHandle, optsExport);
			//	exportHandle = sstats.writeMulti(exportHandle, optsExport);

			//Stats.push_back(std::move(sstats));
		};

		// Validate input files
		vector<string> vinputs;
		for (auto it = inputs.begin(); it != inputs.end(); it++)
		{
			cerr << "Processing " << *it << endl;
			path pi(*it);
			if (!exists(pi)) throw rtmath::debug::xMissingFile(it->c_str());
			if (is_directory(pi))
			{
				path pt = pi / "target.out";
				path ps = pi / "shape.dat";
				boost::shared_ptr<shapefile::shapefile> smain;

				if (exists(ps))
				{
					cerr << " found " << ps << endl;
					smain = boost::shared_ptr<shapefile::shapefile>(new shapefile::shapefile(ps.string()));
					processShape(smain);
				}
				if (doTargetOut && exists(pt))
				{
					cerr << " found " << pi << endl;
					boost::shared_ptr<shapefile::shapefile> s(new shapefile::shapefile(pt.string()));
					if (smain)
						s->tags.insert(std::pair<string, string>("target-src-hash", smain->hash().string()));
					if (smain)
						s->tags.insert(std::pair<string, string>("flake_reference", smain->hash().string()));
					processShape(s);
				}
			} else {
				auto iopts = registry::IO_options::generate();
				iopts->filename(*it);
				try {
					vector<boost::shared_ptr<shapefile::shapefile> > shapes;
					// Handle not needed as the read context is used only once.
					if (shapefile::shapefile::canReadMulti(nullptr, iopts))
						shapefile::shapefile::readVector(nullptr, iopts, shapes);
					else {
						boost::shared_ptr<shapefile::shapefile> s(new shapefile::shapefile);
						s->readFile(*it);
						shapes.push_back(s);
					}
					for (auto &s : shapes)
						processShape(s);
				}
				catch (std::exception &e)
				{
					cerr << e.what() << std::endl;
					continue;
				}
			}
		}

		if (dbcollection->size())
		{
			dHandler = shapefile::shapefile::updateCollection(dbcollection,
				shapefile::shapefile_db_registry::updateType::INSERT_ONLY, dHandler);
			dbcollection->clear();
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

