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
#pragma warning( disable : 4996 )
#pragma warning( disable : 4800 )
#include <cmath>
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <set>

#include <boost/program_options.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/filesystem.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>

#pragma warning( pop ) 
#include "../../rtmath/rtmath/rtmath.h"



int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-shape\n\n";
		rtmath::debug::appEntry(argc, argv);
		//config::parseParams p(argc,argv);

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("trial-run,T", "Only list what would be done")
			("input,i", po::value< vector<string> >(), "input shape files")
			("output,o", "output filename")
			("separate-outputs,s", "Write separate output file for each input. Use default naming scheme.")
			("betas,b", po::value<string>()->default_value("0"), "Specify beta rotations")
			("thetas,t", po::value<string>()->default_value("0:15:90"), "Specify theta rotations")
			("phis,p", po::value<string>()->default_value("0:15:90"), "Specify phi rotations");
		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);    

		if (vm.count("help") || argc == 1) {
			cerr << desc << "\n";
			return 1;
		}

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

		bool sepOutputs = false;
		if (vm.count("separate-outputs")) sepOutputs = true;
		
		string output; 
		if (!sepOutputs)
		{
			if (!vm.count("output"))
			{
				if (inputs.size() == 1)
				{
					output = inputs[0];
					output.append(".stats.xml");
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

		

		if (vm.count("trial-run")) 
		{
			cerr << "Trial run specified. Terminating here." << endl;
			return 0;
		}

		vector<rtmath::ddscat::shapeFileStats> Stats;
		Stats.reserve(inputs.size());

		for (auto it = inputs.begin(); it != inputs.end(); it++)
		{
			// Load the shape file
			cerr << "Processing " << *it << endl;
			rtmath::ddscat::shapefile shp(*it);

			cerr << "\tCalculating baseline statistics" << endl;
			rtmath::ddscat::shapeFileStats sstats(shp);
			sstats.calcStatsBase();
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
			
			if (sepOutputs)
			{
				vector<rtmath::ddscat::shapeFileStats> singleStats;
				singleStats.push_back(std::move(sstats));
				string ofile = *it;
				ofile.append(".stats.xml");
				rtmath::serialization::write<vector<rtmath::ddscat::shapeFileStats> >(singleStats,ofile);
			} else {
				Stats.push_back(std::move(sstats));
			}
		}

		cerr << "Done calculating. Writing results." << endl;
		if (!sepOutputs)
			rtmath::serialization::write<vector<rtmath::ddscat::shapeFileStats> >(Stats,output);
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

