/* shape-extract
* Reads a shape file and writes the desired visualization format
*/

#include <functional>
#include <iostream>
#include <string>
#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/io.h>
#include <Ryan_Debug/registry.h>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/registry.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapefile_supplemental.h"
#include "../../rtmath/rtmath/ddscat/points.h"
#include "../../rtmath/rtmath/error/debug.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-shape-extract\n\n";
		const double pi = boost::math::constants::pi<double>();
		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value< vector<string> >()->multitoken(), "input shape file(s)")
			("output,o", po::value< string >(), "output shape file")
			("export-type", po::value<string>(), "Identifier to export (i.e. shape_points)")
			("decimate", po::value<vector<size_t> >()->multitoken(), "Perform decimation with the given kernel sizing")
			("enhance", po::value<vector<size_t> >()->multitoken(), "Perform enhancement with the given kernel sizing")
			("decimate-threshold", po::value<size_t>(), "Use threshold decimation method")
			("description-append", po::value<string>(), "Apend this to the shape description")
			("convolute", po::value<double>(), "Perform convolution over specified radius")
			("dielScalingFactor", po::value<double>(), "Set dielectric scaling factor for tsv writes. If convolute"
			 " is specified, this can be ignored.")
			("search-radius", po::value<double>(), "Find all points within the specified radius. Must specify x, y, z.")
			("search-nearest", po::value<size_t>(), "Find nearest N points of target.")
			("x,x", po::value<float>()->default_value(0), "x")
			("y,y", po::value<float>()->default_value(0), "y")
			("z,z", po::value<float>()->default_value(0), "z")
			("slice", po::value<int>()->default_value(0), "Slice along plane. Value is axis "
			 "normal (0 - x, 1 - y, 2 - z).")
			("intercept", po::value<float>()->default_value(0), "Intercept for slicing")
			;

		rtmath::debug::add_options(cmdline, config, hidden);

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << "\n";
			if (message.size()) cerr << message << endl;
			exit(1);
		};
		
		rtmath::debug::process_static_options(vm);

		using namespace rtmath::ddscat::shapefile;
		using rtmath::ddscat::shapefile::shapefile;

		if (vm.count("help") || argc == 1) doHelp("");

		if (!vm.count("input")) doHelp("Need to specify input file(s).");
		vector<string> input = vm["input"].as<vector<string> >();

		string sOutput;
		if (vm.count("output")) {
			sOutput = vm["output"].as<string>();
			cerr << "Writing shape file as " << sOutput << endl;
		}
		string exportType;
		if (vm.count("export-type"))
			exportType = vm["export-type"].as<string>();
		std::shared_ptr<Ryan_Debug::registry::IOhandler> handle;
		
		for (const auto &ifile : input)
		{
			cerr << "Reading input shape file " << ifile << endl;
			boost::shared_ptr<shapefile> shp = shapefile::generate(ifile);
			if (vm.count("description-append"))
			{
				std::string da = vm["description-append"].as<std::string>();
				shp->desc.append(da);
			}

			if (vm.count("decimate"))
			{
				vector<size_t> kernel = vm["decimate"].as<vector<size_t> >();
				if (kernel.size() < 3) kernel.assign(3, kernel.at(0));
				decimationFunction df = decimateDielCount;
				if (vm.count("decimate-threshold"))
				{
					size_t threshold = vm["decimate-threshold"].as<size_t>();
					using namespace std::placeholders;
					//rtmath::ddscat::convolutionCellInfo ci;
					df = std::bind(decimateThreshold, std::placeholders::_1,
						std::placeholders::_2, threshold);
				}
				auto dec = shp->decimate(df, kernel[0], kernel[1], kernel[2]);
				shp = dec;
			}

			if (vm.count("enhance"))
			{
				vector<size_t> kernel = vm["enhance"].as<vector<size_t> >();
				if (kernel.size() < 3) kernel.assign(3, kernel.at(0));
				auto dec = shp->enhance(kernel[0], kernel[1], kernel[2]);
				shp = dec;
			}
			float x = vm["x"].as<float>();
			float y = vm["y"].as<float>();
			float z = vm["z"].as<float>();
			if (vm.count("search-nearest")) {
				size_t N = vm["search-nearest"].as<size_t>();
				auto ptsearch = ::rtmath::ddscat::points::points::generate(
					shp->latticePts
					);
				::rtmath::ddscat::points::backend_scalar_type outdists2;
				::rtmath::ddscat::points::backend_index_type outpoints;
				size_t nn = 0; // Number of points found
				nn = ptsearch->nearestNeighbors(N, x, y, z, outpoints, outdists2);
				std::cerr << "There are " << nn << " points found out of max "
					<< N << " near " << x << ", " << y << ", " << z << std::endl;
				std::cerr << "Point\tx\ty\tz\tdistance" << std::endl;
				for (int i=0; i < nn; ++i) {
					std::cerr << outpoints(i,0) << "\t"
						<< shp->latticePts(i,0) << "\t" << shp->latticePts(i,1) << "\t"
						<< shp->latticePts(i,2) << "\t"
						<< std::sqrt(outdists2(i,0)) << std::endl;
				}

			}
			if (vm.count("search-radius")) {
				double radius = vm["search-radius"].as<double>();
				auto ptsearch = ::rtmath::ddscat::points::points::generate(
					shp->latticePts
					);
				::rtmath::ddscat::points::backend_scalar_type outdists2;
				::rtmath::ddscat::points::backend_index_type outpoints;
				size_t nn = 0; // Number of points found
				//nn = ptsearch->nearestNeighbors(4, x, y, z, outpoints, outdists);
				nn = ptsearch->neighborSearchRadius
					((float) (radius*radius), x, y, z, outpoints, outdists2);
				std::cerr << "There are " << nn << " points within rad "
					<< radius << " of " << x << ", " << y << ", " << z << std::endl;
				std::cerr << "The search volume is "
					<< (4.*pi/3.) * std::pow(radius,3.) << std::endl;
				std::cerr << "Point\tx\ty\tz\tdistance" << std::endl;
				for (int i=0; i < nn; ++i) {
					std::cerr << outpoints(i,0) << "\t"
						<< shp->latticePts(i,0) << "\t" << shp->latticePts(i,1) << "\t"
						<< shp->latticePts(i,2) << "\t"
						<< std::sqrt(outdists2(i,0)) << std::endl;
				}

			}
			if (vm.count("convolute")) {
				double radius = vm["convolute"].as<double>();
				auto ptsearch = ::rtmath::ddscat::points::points::generate(
					shp->latticePts
					);
				using namespace std::placeholders;
				decimationFunction df = decimateDielCount;
				df = std::bind(
					::rtmath::ddscat::points::points::convolutionNeighborsRadius,
					std::placeholders::_1,std::placeholders::_2,radius,ptsearch);
				auto cnv = shp->convolute(df, ((size_t) radius) + 1);
				shp = cnv;
			}
			if (vm.count("slice")) {
				int normaxis = vm["slice"].as<int>();
				float intercept = vm["intercept"].as<float>();
				auto cnv = shp->slice(normaxis, intercept, 0.25);
				shp = cnv;
			}

			if (vm.count("output")) {
				auto opts = Ryan_Debug::registry::IO_options::generate();
				opts->filename(sOutput);
				if (exportType.size())
					opts->exportType(exportType);
				if (vm.count("convolute") || vm.count("dielScalingFactor")) {
					double radius = 0;
					if (vm.count("convolute"))
						radius = vm["convolute"].as<double>();
					else
						radius = vm["dielScalingFactor"].as<double>();
					double V = (4.*pi/3.) * std::pow(radius,3.);
					opts->setVal<double>("dielScalingFactor",V);
				}
				handle = shp->writeMulti(handle, opts);
			}
		}
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

