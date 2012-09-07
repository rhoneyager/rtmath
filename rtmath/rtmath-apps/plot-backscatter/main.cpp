#pragma warning( push )
#pragma warning( disable : 4996 ) // Dumb boost uuid warning
#pragma warning( disable : 4800 ) // forcing non-bool type to true or false
#pragma warning( disable : 4521 ) // multiple copy constructors in some PCL stuff
#pragma warning( disable : 4244 ) // warning C4244: '=' : conversion from 'double' to 'float', possible loss of data in FLANN

#include "../../rtmath/rtmath/ROOTlink.h"
#include "../../rtmath/rtmath/VTKlink.h"
#include "../../rtmath/rtmath/MagickLINK.h"

#include <cmath>
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <algorithm>
#include <valarray>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>

#pragma warning( pop ) 
#include "../../rtmath/rtmath/matrixop.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/shapestatsRotated.h"
#include "../../rtmath/rtmath/ddscat/shapestatsviews.h"
#include "../../rtmath/rtmath/serialization.h"
#include "../../rtmath/rtmath/Serialization/shapestats_serialization.h"
#include "../../rtmath/rtmath/error/error.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/matrixop.h"



int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-plot-backscatter\n\n";
		rtmath::debug::appEntry(argc, argv);

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("input,i", po::value< vector<string> >(), "input shape files")

			("dipole-spacing,d", po::value<double>(), "Specify dipole spacing (um)")
			("density", "Output lattice cell density")
			("mass", "Output lattice cell mass (mg)")
			("diameter,D", "Calculate max diameter (in physical units is possible)")
			("aeff-filled", "Calculate effective radius with just shape.dat unit cells")
			("aeff-circumsphere", "Calculate effective radius assuming circumscribing sphere")
			("aeff-V-convex-hull", "Calculate effective radius assuming circumscribing sphere volume")
			("aeff-SA-convex-hull", "Calculate effective radius assuming circumscribing sphere surface area")
			("aeff-V-ellipsoid-max", "Calculate effective radius assuming maximum circumscribing ellipsoid volume")

			("f-circum-sphere", "Calculate volume fraction with circumscribing sphere")
			("f-convex-hull", "Calculate volume fraction with convex hull")
			("f-ellipsoid-max", "Calculate volume fraction with convex hull")

			("PE", "Plot potential energy (in physical units if possible)")

			("convex-hull","Output convex hull information and vtk file")
			("concave-hull", po::value<string>(), "Output concave hull information and vtk file for given value(s)")

			("dipole-density-distance", po::value< string >(), 
			"Make histogram and vtk file of number of neighbors within specified spacings")
			("dipole-density-numneighbors", po::value< string >(),
			"Make histogram and vtk file of rms distance to specified nearest neighbors")

			("betas,b", po::value<string>()->default_value("0"), "Specify beta rotations")
			("thetas,t", po::value<string>()->default_value("0"), "Specify theta rotations")
			("phis,p", po::value<string>()->default_value("0"), "Specify phi rotations");

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);    

		if (vm.count("help") || argc == 1) {
			cerr << desc << "\n";
			return 1;
		}

		double dSpacing = 0;
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

		vector<rtmath::ddscat::shapeFileStats> Stats;
		Stats.reserve(inputs.size());

		for (auto it = inputs.begin(); it != inputs.end(); it++)
		{
			// Load the shape file or shape stats file
			cerr << "Processing " << *it << endl;
			boost::shared_ptr<rtmath::ddscat::shapeFileStats> sstats
				( new rtmath::ddscat::shapeFileStats );
			// Determine file type by extension
			if (it->find(".xml") == std::string::npos)
			{
				cerr << "\tShape file detected\n";
				rtmath::ddscat::shapefile shp(*it);
				rtmath::ddscat::shapeFileStats *sp = 
					new rtmath::ddscat::shapeFileStats(shp);
				sstats.reset( sp );
				cerr << "\tCalculating baseline statistics" << endl;
				sstats->calcStatsBase();
				for (auto beta = betas.begin(); beta != betas.end(); beta++)
				{
					for (auto theta = thetas.begin(); theta != thetas.end(); theta++)
					{
						for (auto phi = phis.begin(); phi != phis.end(); phi++)
						{
							cerr << "\tCalculating rotation (beta,theta,phi): ("
								<< *beta << ", " << *theta << ", " << *phi << ")" << endl;
							sstats->calcStatsRot(*beta,*theta,*phi);
						}
					}
				}
			} else {
				cerr << "\tStats file detected\n";
				if (!vm.count("separate-outputs"))
				{
					rtmath::serialization::read<rtmath::ddscat::shapeFileStats>
						(*sstats, *it);
				} else {
					// stats are in a vector (old method)
					vector<rtmath::ddscat::shapeFileStats> vs;
					rtmath::serialization::read<vector<rtmath::ddscat::shapeFileStats> >
						(vs, *it);
					if (vs.size())
					{
						*sstats = vs[0];
					} else {
						throw rtmath::debug::xUnknownFileFormat(it->c_str());
					}
				}

			}

			boost::shared_ptr<rtmath::ddscat::shapeFileStatsDipoleView> sstatsView
				(new rtmath::ddscat::shapeFileStatsDipoleView(sstats, dSpacing));

			if (vm.count("diameter"))
			{
				cout << "Diameter (d): " << sstats->max_distance << endl;
				if (dSpacing)
					cout << "Diameter (um): " << sstatsView->max_distance() << endl;
			}


			//cerr << "Done." << endl;
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

