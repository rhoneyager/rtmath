/* shape
 * A program designed for analyzing ddscat shape.par files and the associated ___ files output when 
 * ddscat runs. The shape file is loaded, and various properties of the shape are analyzed. 
 * In particular, the moments of inertia are calculated for the shape, and its most likely 
 * axis, given lamellar-like flow conditions, is calculated. This is important because it 
 * indicates if the shapes that are generated contain the appropriate angle calculations!
 * 
 * Also, ancillary useful quantities are calculated. Crystal mass is not, as these shapes may be scaled.
 * However, the number of dipoles is presented, along with crystal volume relations for a given 
 * radius. 
 */
#pragma warning( disable : 4503 ) // decorated length exceeded
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
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/kurtosis.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/skewness.hpp>
#include <boost/accumulators/statistics/variance.hpp>


#include <Ryan_Debug/debug.h>
#include <Ryan_Serialization/serialization.h>
#pragma warning( pop ) 

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"

#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

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
		Ryan_Serialization::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("trial-run,T", "Only list what would be done")
			("dipole-spacing,d", po::value<double>(), "Set dipole spacing for file exports.")
			("input,i", po::value< vector<string> >(), "Input shape files")
			("output,o", po::value<string>(), "Output filename")
			("export-type", po::value<string>(), "Identifier to export (i.e. ar_rot_data)")
			("export,e", po::value<string>(), "Export filename (all shapes are combined into this)")
			//("separate-outputs,s", "Vestigial option. Write separate output file for each input. Use default naming scheme.")
			;

		rtmath::debug::add_options(cmdline, config, hidden);

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		if (vm.count("help") || argc == 1) {
			cerr << desc << "\n";
			return 1;
		}
		
		rtmath::debug::process_static_options(vm);
		Ryan_Serialization::process_static_options(vm);
		ddscat::stats::shapeFileStats::process_static_options(vm);

		const size_t nExts = 1;
		const char* exportExts[nExts] = { "tsv" };
		rtmath::registry::genAndRegisterIOregistryPlural
			<::rtmath::ddscat::stats::shapeFileStats,
			rtmath::ddscat::stats::shapeFileStats_IO_output_registry>(
			nExts, exportExts, "shape-tsv-ars", "ar_rot_data");


		if (vm.count("dipole-spacing"))
			dSpacing = vm["dipole-spacing"].as<double>();

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

		

		// Validate input files
		for (auto it = inputs.begin(); it != inputs.end(); it++)
		{
			path pi(*it);
			if (!exists(pi)) throw rtmath::debug::xMissingFile(it->c_str());
			if (is_directory(pi))
			{
				path pt = pi / "target.out";
				path ps = pi / "shape.dat";
				if (exists(pt)) *it = pt.string();
				else if (exists(ps)) *it = ps.string();
				else throw rtmath::debug::xPathExistsWrongType(it->c_str());
			}
		}

		if (vm.count("trial-run")) 
		{
			cerr << "Trial run specified. Terminating here." << endl;
			return 0;
		}

		vector<rtmath::ddscat::stats::shapeFileStats> Stats;
		Stats.reserve(inputs.size());

		for (auto it = inputs.begin(); it != inputs.end(); it++)
		{
			// Load the shape file
			cerr << "Processing " << *it << endl;
			rtmath::ddscat::shapefile::shapefile shp(*it);

			cerr << "\tCalculating baseline statistics" << endl;
			rtmath::ddscat::stats::shapeFileStats sstats(shp);
			//sstats.calcStatsBase();
			/*
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
			*/

			Stats.push_back(std::move(sstats));
		}

		std::cerr << "Done calculating. Writing results." << endl;

		// Need to handle serialization versus plugins properly here.
		
		std::shared_ptr<registry::IOhandler> handle, exportHandle;
		for (const auto &s : Stats)
		{
			if (output.size())
				handle = s.writeMulti(s._shp->filename.c_str(), handle, output.c_str());
			if (doExport)
				exportHandle = s.writeMulti(s._shp->filename.c_str(), exportHandle,
					exportFilename.c_str(), "", exportType.c_str());
		}
		//::Ryan_Serialization::write<vector<rtmath::ddscat::stats::shapeFileStats> >(Stats,output);
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

namespace rtmath {
	namespace registry {

		const std::string PLUGINID_ARS("shape-tsv-ars");
		using std::shared_ptr;

		struct tsv_ar_handle : public rtmath::registry::IOhandler
		{
			tsv_ar_handle(const char* filename, IOtype t) : IOhandler(PLUGINID_ARS) { open(filename, t); }
			virtual ~tsv_ar_handle() {}
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
					"aeff_um\tNumber of Rotations\t"
					"min_as_abs_xy\tmin_as_abs_xz\tmin_as_abs_yz\t"
					"max_as_abs_xy\tmax_as_abs_xz\tmax_as_abs_yz\t"
					"mean_as_abs_xy\tmean_as_abs_xz\tmean_as_abs_yz\t"
					"skewness_as_abs_xy\tskewness_as_abs_xz\tskewness_as_abs_yz\t"
					"kurtosis_as_abs_xy\tkurtosis_as_abs_xz\tkurtosis_as_abs_yz\t"
					"variance_as_abs_xy\tvariance_as_abs_xz\tvariance_as_abs_yz\n"
					;
			}
			std::shared_ptr<std::ofstream> file;
		};

		shared_ptr<IOhandler>
			export_tsv_ar_rot_data
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			const rtmath::ddscat::stats::shapeFileStats *s)
		{
				std::string filename = opts->filename();
				IOhandler::IOtype iotype = opts->iotype();

				using std::shared_ptr;
				std::shared_ptr<tsv_ar_handle> h;
				if (!sh)
				{
					h = std::shared_ptr<tsv_ar_handle>(new tsv_ar_handle(filename.c_str(), iotype));
				}
				else {
					if (sh->getId() != PLUGINID_ARS) RTthrow debug::xDuplicateHook("Bad passed plugin");
					h = std::dynamic_pointer_cast<tsv_ar_handle>(sh);
				}

				double aeff_um = s->Scircum_sphere.aeff_V * dSpacing;
				size_t nRots = s->rotations.size();

				// Write the aspect ratio stat data
				// Go through the loaded rotations and calculate ar stats

				using namespace boost::accumulators;
				//using namespace boost::accumulators::tag;

				// Do two passes to be able to renormalize coordinates
				accumulator_set<double, boost::accumulators::stats<
					tag::mean, tag::min, tag::max, 
					tag::skewness,
					tag::kurtosis,
					tag::variance
				> > m_xy, m_yz, m_xz;

				for (const auto &rot : s->rotations)
				{
					m_xy(rot->as_abs(0, 1));
					m_yz(rot->as_abs(1, 2));
					m_xz(rot->as_abs(0, 2));
				}

				
				(*(h->file.get())) << s->_shp->hash().lower << "\t" << s->V_dipoles_const << "\t"
					<< aeff_um << "\t" << nRots << "\t"
					<< ::boost::accumulators::min(m_xy) << "\t" << ::boost::accumulators::min(m_yz) << "\t" << ::boost::accumulators::min(m_xz) << "\t"
					<< ::boost::accumulators::max(m_xy) << "\t" << ::boost::accumulators::max(m_yz) << "\t" << ::boost::accumulators::max(m_xz) << "\t"
					<< boost::accumulators::mean(m_xy) << "\t" << boost::accumulators::mean(m_yz) << "\t" << boost::accumulators::mean(m_xz) << "\t"
					<< boost::accumulators::skewness(m_xy) << "\t" << boost::accumulators::skewness(m_yz) << "\t" << boost::accumulators::skewness(m_xz) << "\t"
					<< boost::accumulators::kurtosis(m_xy) << "\t" << boost::accumulators::kurtosis(m_yz) << "\t" << boost::accumulators::kurtosis(m_xz) << "\t"
					<< boost::accumulators::variance(m_xy) << "\t" << boost::accumulators::variance(m_yz) << "\t" << boost::accumulators::variance(m_xz)
					<< std::endl;
				
				return h; // Pass back the handle
		}

		template<> shared_ptr<IOhandler> write_file_type_multi<rtmath::ddscat::stats::shapeFileStats>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			const rtmath::ddscat::stats::shapeFileStats *s)
		{
				std::string exporttype = opts->exportType();
				if (exporttype == "ar_rot_data") return export_tsv_ar_rot_data(sh, opts, s);
				else { RTthrow debug::xUnimplementedFunction(); }
				return nullptr;
		}
	}
}
