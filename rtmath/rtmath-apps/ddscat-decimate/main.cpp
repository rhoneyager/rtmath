/* This program reads in a shape file and decimates / expands the grid by the 
* specified contraction / dilation factor. It also can write diel.tab-style 
* files for the different possible cases of filled / unfilled cells, using 
* a variety of techniques.
*/

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <iostream>
#include <sstream>
//#include <list>
#include <vector>
//#include <cstdlib>

#include <Ryan_Debug/debug.h>
#include <Ryan_Serialization/serialization.h>

//#include <pcl/point_cloud.h>
//#include <pcl/octree/octree.h>

#include "../../rtmath/rtmath/denseMatrix.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/dielTabFile.h"
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/common_templates.h" // gives paramSet
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/error/error.h"

int main(int argc, char** argv)
{
	using namespace std;
	try {
		namespace po = boost::program_options;

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("input,i", po::value<string >(), 
			"Specify input shape file.")
			/*("inputddpar,p", po::value<string>(),
			"Specify input ddscat.par file")
			//("inputdir,d", po::value<string>(),
			//"Specify input directory")
			//("base-dipole-spacing", po::value<double>(), 
			//"Specify the base dipole spacing (microns)")*/
			("output,o", po::value<string>()->default_value("./decimated.shp"),
			"Specify output file")
			//("twolevels", po::value<unsigned int>(), "Give output shape only two levels, "
			//"with the resultant cells being filled or not using a threshold.")
			("factor,f", po::value<unsigned int>()->default_value(2),
			"scaling factor (in each direction)")
			;

		po::positional_options_description p;
		p.add("input",1);
		p.add("output",2);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);

		auto doHelp = [&desc](const std::string &message)
		{
			std::cerr << message << std::endl;
			std::cerr << desc << std::endl;
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");
		if (!vm.count("input")) doHelp("Need to specify input file\n");

		string input = vm["input"].as<string>();
		string output = vm["output"].as<string>();

		unsigned int scale = vm["factor"].as<unsigned int>();
		if (!scale) doHelp("factor must be an integer greater than zero");

		bool expand = (vm.count("expand")) ? true : false;

		//bool hasThreshold = (vm.count("twolevels")) ? true : false;
		//int threshold = 0;
		//if (hasThreshold) threshold = vm["twolevels"].as<unsigned int>();

		// TODO: add in diel commands here

		namespace files = boost::filesystem;

		// Verify input file existence
		if (!files::exists(files::path(input))) doHelp("Input file does not exist");
		if (files::is_directory(files::path(input))) doHelp("Input file is a directory");
		//if (files::exists(files::path(output)))

		using namespace rtmath::ddscat;

		// Open the input file
		boost::shared_ptr<const shapefile> shp = boost::shared_ptr<const shapefile>(
			new rtmath::ddscat::shapefile(input));

		// Find min / max of points using shapestats
		//boost::shared_ptr<const shapeFileStats> sstats = boost::shared_ptr<const shapeFileStats>(
		//	new shapeFileStats(shp));

		boost::shared_ptr<shapefile> shpout = shp->decimate((size_t) scale);

		using namespace boost::accumulators;
		// Finding the new mean, the easier way.
		accumulator_set<float, stats<tag::mean> > m_x, m_y, m_z;

		for (size_t i=0; i<shpout->numPoints; ++i)
		{
			m_x(shpout->latticePts(i,0));
			m_y(shpout->latticePts(i,1));
			m_z(shpout->latticePts(i,2));
		}

		// Find min / max of points using shapestats
		//rtmath::ddscat::shapeFileStats sstatsout(shpout);
		//shpout->x0 = sstatsout.b_mean;
		//shpout->x0 = shp->x0;
		shpout->x0(0) = boost::accumulators::mean(m_x);
		shpout->x0(1) = boost::accumulators::mean(m_y);
		shpout->x0(2) = boost::accumulators::mean(m_z);

		shpout->write(output);

		// Calculate dielectric information using the 
		// selected dielectric pattern

		// Write the dielectric files

		// Write the new ddscat.par file (referencing new dielectrics)

		// Make a vtk file of the decimated shape

	} catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}
