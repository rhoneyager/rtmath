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
#include "../../rtmath/rtmath/error/debug.h"

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
			("input,i", po::value< vector<string> >(), "input shape files")
			("output,o", po::value<string>(), "output filename")
			//("separate-outputs,s", "Vestigial option. Write separate output file for each input. Use default naming scheme.")
			;

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

		Ryan_Serialization::process_static_options(vm);
		ddscat::stats::shapeFileStats::process_static_options(vm);

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
		bool sepOutputs = true;
		//if (vm.count("separate-outputs")) sepOutputs = true;

		string output; 
		if (!sepOutputs)
		{
			if (!vm.count("output"))
			{
				if (inputs.size() == 1)
				{
					output = inputs[0];
					output.append(".stats.xml");
					sepOutputs = true; // Set this because we do not want a vector with size 1!
				} else {
					output = "shapestats.xml";
				}
			} else {
				output = vm["output"].as<string>();
			}

			cerr << "Outputting to: " << output << endl;
		} else {
			cerr << "Separating output files" << endl;
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

		// Specify beta, theta, phi rotations
		/*
		string sbeta = vm["betas"].as<string>();
		string stheta = vm["thetas"].as<string>();
		string sphi = vm["phis"].as<string>();

		paramSet<double> betas(sbeta);
		paramSet<double> thetas(stheta);
		paramSet<double> phis(sphi);
		cerr << "Betas: ";
		for (auto it = betas.begin(); it != betas.end(); it++)
			cerr << *it << " ";
		cerr << endl;
		cerr << "Thetas: ";
		for (auto it = thetas.begin(); it != thetas.end(); it++)
			cerr << *it << " ";
		cerr << endl;
		cerr << "Phis: ";
		for (auto it = phis.begin(); it != phis.end(); it++)
			cerr << *it << " ";
		cerr << endl;
		*/


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

			if (sepOutputs)
			{
				//vector<rtmath::ddscat::shapeFileStats> singleStats;
				//singleStats.push_back(std::move(sstats));
				string ofile = *it;
				ofile.append(".stats.xml");
				::Ryan_Serialization::write<rtmath::ddscat::stats::shapeFileStats >(sstats,ofile);
			} else {
				Stats.push_back(std::move(sstats));
			}
		}

		cerr << "Done calculating. Writing results." << endl;
		if (!sepOutputs)
			::Ryan_Serialization::write<vector<rtmath::ddscat::stats::shapeFileStats> >(Stats,output);
		//shp.print(out);
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

