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
#include <Ryan_Serialization/serialization.h>
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

		cmdline.add_options()
			("help,h", "produce help message")
			("trial-run,T", "Only list what would be done")
			("dipole-spacing,d", po::value<double>(), "Set dipole spacing for file exports.")
			("input,i", po::value< vector<string> >(), "Input shape files")
			("output,o", po::value<string>(), "Output filename")
			("export-type", po::value<string>(), "Identifier to export (i.e. ar_rot_data)")
			("export,e", po::value<string>(), "Export filename (all shapes are combined into this)")
			("tag", po::value<vector<string> >(), "Using \"key=value pairs\", add tags to the output (not with .shp files)")
			//("separate-outputs,s", "Vestigial option. Write separate output file for each input. Use default naming scheme.")
			("process-target-out", po::value<bool>()->default_value(0), "Process target.out files in addition to shape.dat files.")
			("hash-shape", "Store shapefile hash")
			("hash-stats", "Store shapefile hash")
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

		// Validate input files
		vector<string> vinputs;
		for (auto it = inputs.begin(); it != inputs.end(); it++)
		{
			path pi(*it);
			if (!exists(pi)) throw rtmath::debug::xMissingFile(it->c_str());
			if (is_directory(pi))
			{
				path pt = pi / "target.out";
				path ps = pi / "shape.dat";
				if (doTargetOut)
					if (exists(pt)) vinputs.push_back(pt.string());
				if (exists(ps)) vinputs.push_back(ps.string());
				else continue;
				//else throw rtmath::debug::xPathExistsWrongType(it->c_str());
			} else vinputs.push_back(*it);
		}

		if (vm.count("trial-run")) 
		{
			cerr << "Trial run specified. Terminating here." << endl;
			return 0;
		}

		//vector<rtmath::ddscat::stats::shapeFileStats> Stats;
		//Stats.reserve(inputs.size());

		std::shared_ptr<registry::IOhandler> handle, exportHandle;

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

		using std::vector;
		using namespace rtmath::ddscat;
		vector<boost::shared_ptr<shapefile::shapefile> > shapes;
		for (auto it = vinputs.begin(); it != vinputs.end(); ++it)
		{
			cerr << "Processing " << *it << endl;
			auto iopts = registry::IO_options::generate();
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

		for (const auto &shp : shapes)
		{
			cerr << "Shape " << shp->hash().lower << endl;
			for (auto &t : tags)
				shp->tags.insert(t);

			if (vm.count("hash-shape")) shp->writeToHash();
			//if (vm.count("hash-stats")) stats->writeToHash();
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

			//cerr << "\tCalculating statistics" << endl;
			//rtmath::ddscat::stats::shapeFileStats sstats(shp);
			
			if (output.size())
				handle = shp->writeMulti(handle, opts);
			//	handle = sstats.writeMulti(handle, opts);
			if (doExport)
				exportHandle = shp->writeMulti(exportHandle, optsExport);
			//	exportHandle = sstats.writeMulti(exportHandle, optsExport);

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

