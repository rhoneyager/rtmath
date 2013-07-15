/* This program is designed to find optimum scaling factors for tmatrix runs 
* for various particles. The optimum values are determined using ddscat run 
* results as a base, and Google's Ceres Solver is used to minimize the 
* defined cost function.
*
* This is designed to perform runs with medium-sized numbers of particles, as 
* repeated t-matrix runs may take some time. All results are written out 
* to csv files. The overall cost function is minimized in 3d, and gridded 
* values may also be calculated and exported.
*
* Also designed to export results for individual flakes.
*/

#include <vector>
#include <boost/serialization/vector.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/ddscat/tmData.h"
#include <Ryan_Serialization/serialization.h>
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/shapestatsRotated.h"
//#include "../../rtmath/rtmath/ddscat/shapestatsviews.h" /// \todo Fix shapestatsviews?

#include "dataset.h"
#include "run.h"

#include "../../rtmath/rtmath/error/error.h"

int main(int argc, char** argv)
{
	using namespace std;
	try {
		namespace po = boost::program_options;

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("inputs,i", po::value<vector<string> >(), 
			"Specify input files (.avg, .shp files). "
			"If a folder is speified, recurse and select all matching files.")
			("output,o", po::value<string>()->default_value("tmminres.xml"),
			"Output file for run results")
			("write-indiv-flakes", "Write output for individual flakes")
			("grid-output", "Perform gridded calculations in addition to pure minimization")
			("cost-function,c", po::value<string>()->default_value("both"),
			"Select the cost function that will be used for minimization. Can be: qsca, qbksc or both.")
			("use-volfrac", po::value<bool>()->default_value(true),
			"Use volume fraction in minimization")
			("use-aeff", po::value<bool>()->default_value(true),
			"Use effective radius in minimization")
			//("use-ar", po::value<bool>()->default_value(false),
			//"Use aspect ratio in minimization")
			("stats-dir", po::value<string>(),
			"If only a shape file is specified, write stats to this directory. Used for speedups in future runs.")
			;

		po::positional_options_description p;
		p.add("inputs",-1);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);

		vector<string> rawinputs; // May be expanded by os.

		auto doHelp = [&](const std::string &message)
		{
			cerr << message << endl;
			cerr << desc << endl;
			exit(1);
		};
		if (vm.count("help") || argc == 1) doHelp("");

		if (!vm.count("inputs")) doHelp("Need to specify input files.");
		rawinputs = vm["inputs"].as<vector<string> >();

		bool writeStats = false;
		string statsDir;
		if (vm.count("stats-dir"))
		{
			writeStats = true;
			statsDir = vm["stats-dir"].as<string>();
		}

		string output = vm["output"].as<string>();

		using namespace boost::filesystem;

		map<string,dataset> data;

		auto insertMapping = [&](const boost::filesystem::path &p)
		{
			if (dataset::isValid(p) == false) return;
			std::string prefix = dataset::getPrefix(p);
			if (!data.count(prefix))
				data[prefix] = std::move(dataset(prefix));
			if (dataset::isAvg(p))
				data[prefix].ddres.push_back(p);
			if (dataset::isShape(p))
				data[prefix].shapefile = p;
			if (dataset::isShapeStats(p))
				data[prefix].shapestatsfile = p;
		};

		auto expandSymlinks = [](const boost::filesystem::path &p) -> boost::filesystem::path
		{
			using namespace boost::filesystem;
			if (is_symlink(p))
			{
				path pf = boost::filesystem::absolute(read_symlink(p), p.parent_path());
				if (is_directory(pf))
					return pf;
				else
					return p;
			} else {
				return p;
			}
		};

		// Expand directories and validate input files
		// If a directory is specified, recurse through the 
		// structure and pick up all valid shape files.
		for (auto it = rawinputs.begin(); it != rawinputs.end(); ++it)
		{
			path pi(*it);
			pi = expandSymlinks(pi);
			if (!exists(pi)) throw rtmath::debug::xMissingFile(it->c_str());
			if (is_directory(pi) )
			{
				vector<path> cands;
				copy(recursive_directory_iterator(pi,symlink_option::recurse), 
					recursive_directory_iterator(), back_inserter(cands));
				for (auto f = cands.begin(); f != cands.end(); ++f)
				{
					path pf = *f;
					pf = expandSymlinks(pf);
					insertMapping(pf); // already does dir checking
				}
			} else {
				insertMapping(*it);
			}
		}

		ddPoint::ddPointSet ddPoints;
		for (auto it = data.begin(); it != data.end(); ++it)
		{
			cerr << "Processing " << it->first << endl;
			// Skip if there are no ddscat results to use
			if (it->second.ddres.size() == 0) continue;
			// Skip if no shape or shape stats file
			if (it->second.shapefile.empty() && it->second.shapestatsfile.empty()) continue;

			// If no shape stats file, generate one
			// And place generated stats file in specified directory
			// And read the stats results
			it->second.prepStats(writeStats,statsDir);
			// Stats results give volume fraction

			// Read the ddscat results
			// A cache is created for each object to prevent multiple loads.
			it->second.ddloaded.resize(it->second.ddres.size());
			for (size_t i=0; i< it->second.ddres.size(); ++i)
			{
				// Three file types:
				// - raw ddscat output
				// - compressed ddscat output
				// - serialized ddscat output
				// All typed are now handled by the ddOutputSingle read function.
				it->second.ddloaded[i] = boost::shared_ptr<rtmath::ddscat::ddOutputSingle>
					(new rtmath::ddscat::ddOutputSingle(it->second.ddres[i].string()));
			}

			// Everything is now loaded for this datum. Add it to the run.

			ddPoint::ddPointSet ddPointSetData = ddPoint::genFromDataset(it->second);
			ddPoints.insert(ddPoints.end(), ddPointSetData.begin(), ddPointSetData.end());
		}

		// Create an object to store all of the tm results
		tmRunSet tmRuns;
		tmRuns.ddPoints = ddPoints;

		// Perform the iteration using ceres solver
		// For rach iteration, add to tmRuns.tmRuns.
		// For minima, set min_.


		/// \todo Finish this using ceres solver





		// Write all of the data
		Ryan_Serialization::write<tmRunSet>(tmRuns, output, "tmRunSet");

		/*
		if (pt->data.size())
		{
			auto qt = pt->data.begin();
			gout << it->first << ",iso,iso,iso,iso,";
			gout << freq << "," << ddfile.wave() << "," << pt->sizep << ","
				<< dSpacing << "," << pt->T << "," << pt->nu << "," 
				<< pt->reff.real() << "," << pt->reff.imag() << ","
				<< pt->volMeth << "," << pt->dielMeth << "," << pt->shapeMeth << ","
				<< aeff << "," << qt->get()->tm->base->axi << ","
				<< qt->get()->tm->base->eps << ","
				<< qext << "," << tQextm << ","
				<< qsca << "," << tQscam << "," 
				<< qbk << "," << tQbkm << ","
				<< qsca/qext << "," << tQscam/tQextm;

			gout << std::endl;
		}
		*/

	}
	catch (std::exception &e)
	{
		cerr << "Exception caught\n";
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

