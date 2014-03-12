/* shape-hull
* This program is designed to take a shapefile and write a vtk file corresponding 
* to either the initial points or the convex hull.
*/
#pragma warning( push )
#pragma warning( disable : 4996 ) // Dumb boost uuid warning
#pragma warning( disable : 4800 ) // forcing non-bool type to true or false
#include <cmath>
#include <memory>
#include <iostream>
#include <fstream>
#include <ios>
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
#include "../../rtmath/rtmath/ddscat/hulls.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/registry.h"
#include "../../rtmath/rtmath/error/debug.h"

namespace rtmath { namespace plugins { namespace internal { namespace tsv { void registerTSV(); } } } }

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-shape-hull\n\n";

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", 1);
		p.add("output", 2);

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		//ddscat::shapeFileStats::add_options(cmdline, config, hidden);
		//Ryan_Serialization::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value< vector<string> >()->multitoken(), "Input shape file(s)")
			("output,o", po::value<vector<string> >(), "Output file(s). Each input is written to all of the outputs.")
			//("convex-hull,c", "Calculate and write convex hull")
			;
		rtmath::debug::add_options(cmdline, config, hidden);
		rtmath::ddscat::stats::shapeFileStats::add_options(cmdline, config, hidden);

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);
		rtmath::ddscat::stats::shapeFileStats::process_static_options(vm);
		rtmath::plugins::internal::tsv::registerTSV();


		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << endl;
			cerr << message << endl;
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");

		//Ryan_Serialization::process_static_options(vm);
		//ddscat::shapeFileStats::process_static_options(vm);

		bool convex = false;
		if (vm.count("convex-hull")) convex = true;

		if (!vm.count("input"))
			doHelp("Need to specify input file(s).");
		vector<string> inputs = vm["input"].as< vector<string> >();

		if (!vm.count("output"))
			doHelp("Need to specify output file(s).");
		vector<string> output = vm["output"].as< vector<string> >();
		std::vector<std::shared_ptr<rtmath::registry::IOhandler> > outputios(output.size());

		for (const auto &input : inputs)
		{
			// Validate input files
			path pi(input);
			if (!exists(pi)) 
				throw rtmath::debug::xMissingFile(input.c_str());
			if (is_directory(pi)) 
				throw rtmath::debug::xPathExistsWrongType(input.c_str());

			using rtmath::ddscat::shapefile::shapefile;
			using rtmath::ddscat::stats::shapeFileStats;
			boost::shared_ptr<shapefile> shp;
			boost::shared_ptr<shapeFileStats> stats;
			// Load the shape file
			try {

				using namespace rtmath::Voronoi;
				shp = boost::shared_ptr<shapefile> (new shapefile(input));
				stats = shapeFileStats::genStats(shp);
				// stats already generates this
				auto vd = shp->generateVoronoi("standard", VoronoiDiagram::generateStandard);
				auto cvxCands = vd->calcCandidateConvexHullPoints();
				shp->latticeExtras["cvxCands"] = cvxCands;
				auto SAfracExternal = vd->calcPointsSAfracExternal();
				shp->latticeExtras["SAfracExternal"] = SAfracExternal;
				auto depth = vd->calcSurfaceDepth();
				shp->latticeExtras["SurfaceDepth"] = depth;
				auto depthSrc = vd->calcSurfaceDepthVectors();
				shp->latticeExtras["SurfaceDepthVectors"] = depthSrc;
				auto depthNeighs = vd->calcSurfaceNumNeighs();
				shp->latticeExtras["SurfaceDepthNNeighs"] = depthNeighs;
				auto depthOrder = vd->calcSurfaceFillingOrder();
				shp->latticeExtras["SurfaceFillingOrder"] = depthOrder;

				for (size_t i=0; i < output.size(); ++i)
				{
					try {
						outputios[i] = stats->writeMulti("", outputios[i], output[i].c_str());
						outputios[i] = vd->writeMulti("", outputios[i], output[i].c_str());
					} catch (rtmath::debug::xUnknownFileFormat &e)
					{
						std::cerr << "Error: unknown file format when writing " << output[i] << endl;
						std::cerr << e.what() << endl;
					}
				}
			} catch (std::exception &e)
			{
				std::cerr << "Error processing the file: " << input << endl;
				std::cerr << e.what() << std::endl;
				continue;
			} catch (...)
			{
				std::cerr << "Error processing the file: " << input << endl;
				continue;
			}

		}
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

// A custom tsv writer is being used here - it's similar to how a plugin does it.
// This enables writing multiple tsv output files.
namespace rtmath
{
	namespace plugins
	{
		namespace internal
		{
			namespace tsv
			{
				const std::string PLUGINID("shape-hull-tsv");
				struct tsv_handle : public rtmath::registry::IOhandler
				{
					tsv_handle(const char* filename, IOtype t) : IOhandler(PLUGINID) { open(filename, t); }
					virtual ~tsv_handle() {}
					void open(const char* filename, IOtype t)
					{
						using namespace boost::filesystem;
						switch (t)
						{
						case IOtype::EXCLUSIVE:
						case IOtype::DEBUG:
						case IOtype::READONLY:
							throw;
							break;
						case IOtype::CREATE:
							if (exists(path(filename))) throw("File already exists");
						case IOtype::TRUNCATE:
							file = std::shared_ptr<std::ofstream>(new std::ofstream(filename, std::ios_base::trunc));
							writeHeader();
							break;
						case IOtype::READWRITE:
							{
								bool e = false;
								if (exists(path(filename))) e = true;
								file = std::shared_ptr<std::ofstream>(new std::ofstream(filename, std::ios_base::app));
								if (!e) writeHeader(); // If the file had to be created, give it a header
							}
							break;
						}
					}
					void writeHeader()
					{
						(*(file.get())) << "Hash\tV_dipoles_const\t"
							"Circum_Sphere_V\tCircum_Sphere_SA\t"
							"Convex_V\tConvex_SA\t"
							"Voronoi_V\tVoronoi_SA\t"
							"as_abs_xy\tas_abs_xz\tas_abs_yz\t"
							"as_rms_xy\tas_rms_xz\tas_rms_yz\t"
							"as_abm_xy\tas_abm_xz\tas_abm_yz\n"
							;
					}
					std::shared_ptr<std::ofstream> file;
				};


				std::shared_ptr<rtmath::registry::IOhandler> write_tsv_multi_shapestats
					(std::shared_ptr<rtmath::registry::IOhandler> sh, 
					const char* filename, 
					const rtmath::ddscat::stats::shapeFileStats *s, 
					const char* key, // Unused for this type of write
					rtmath::registry::IOhandler::IOtype iotype)
				{
					using std::shared_ptr;
					std::shared_ptr<tsv_handle> h;
					if (!sh)
						h = std::shared_ptr<tsv_handle>(new tsv_handle(filename, iotype));
					else {
						if (sh->getId() != "shape-hull-tsv") RTthrow debug::xDuplicateHook("Bad passed plugin");
						h = std::dynamic_pointer_cast<tsv_handle>(sh);
					}

					// Initial file creation handles writing the initial header.
					// So, just write the data.
					auto r = s->calcStatsRot(0,0,0);
					(*(h->file.get())) << s->_shp->hash().lower << "\t" << s->V_dipoles_const << "\t"
						<< s->Scircum_sphere.V << "\t" << s->Scircum_sphere.SA << "\t"
						<< s->Sconvex_hull.V << "\t" << s->Sconvex_hull.SA << "\t"
						<< s->SVoronoi_hull.V << "\t" << s->SVoronoi_hull.SA << "\t"
						<< r->as_abs(0,1) << "\t" << r->as_abs(0,2) << "\t" << r->as_abs(1,2) << "\t"
						<< r->as_rms(0,1) << "\t" << r->as_rms(0,2) << "\t" << r->as_rms(1,2) << "\t"
						<< r->as_abs_mean(0,1) << "\t" << r->as_abs_mean(0,2) << "\t" << r->as_abs_mean(1,2)
						<< std::endl;
						;

					return h; // Pass back the handle
				}

				/// \todo Make a template for this
				bool match_tsv(const char* filename, const char* type)
				{
					using namespace boost::filesystem;
					using std::string;
					using std::ofstream;

					string stype(type);
					path pPrefix(filename);
					if (stype == "tsv" || stype == ".tsv") return true;
					else if (pPrefix.extension() == ".tsv") return true;
					return false;
				}

				/// \todo Make a template for this
				bool match_tsv_multi(const char* filename, const char* type, std::shared_ptr<rtmath::registry::IOhandler> h)
				{
					if (h)
					{
						if (h->getId() != PLUGINID) return false;
						return true;
					} else {
						return match_tsv(filename, type);
					}
				}

				void registerTSV()
				{
					static rtmath::registry::IO_class_registry<
						::rtmath::ddscat::stats::shapeFileStats> s2;
					s2.io_matches = match_tsv;
					s2.io_processor = nullptr;
					s2.io_multi_matches = match_tsv_multi;
					s2.io_multi_processor = write_tsv_multi_shapestats;
					rtmath::ddscat::stats::shapeFileStats::usesDLLregistry<
						rtmath::ddscat::stats::shapeFileStats_IO_output_registry,
						rtmath::registry::IO_class_registry<::rtmath::ddscat::stats::shapeFileStats> >
						::registerHook(s2);
				}

			}
		}
	}
}
