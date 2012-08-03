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
#include <boost/shared_ptr.hpp>

#pragma warning( pop ) 
#include "../../rtmath/rtmath/rtmath.h"
#include "../../rtmath/rtmath/ROOTlink.h"
#include "../../rtmath/rtmath/Garrett/image.h"
#include "../../rtmath/rtmath/PCLlink.h"


int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-garrett\n\n";
		rtmath::debug::appEntry(argc, argv);

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("input,i", po::value< vector<string> >(), "input files (png, vtk, pcd)")
			("output-vtk", "Produce vtk outputs")
			("output-surf", "Produce contour plot of surface")
			("output-zhist", "Produce histogram of pixel brightness temperatures")
//			("output-gradzhist", "Produce histogram of gradient of brightness")
			("output-pcd", "Produce pcd outputs");

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

		// Validate input files
		for (auto it = inputs.begin(); it != inputs.end(); it++)
		{
			path pi(*it);
			if (!exists(pi)) throw rtmath::debug::xMissingFile(it->c_str());
			if (is_directory(pi))
			{
				throw rtmath::debug::xPathExistsWrongType(it->c_str());
			}
		}

		vector<rtmath::Garrett::image> images;
		images.reserve(inputs.size());

		for (auto it = inputs.begin(); it != inputs.end(); it++)
		{
			// Load the shape file or shape stats file
			cerr << "Processing " << *it << endl;
			rtmath::Garrett::image img;
			img.read(*it);

			if (vm.count("output-vtk"))
				img.writeVTKpoints(string(*it).append(".vtk"));
			if (vm.count("output-pcd"))
				img.writePCD(string(*it).append(".pcd"));
			if (vm.count("output-surf"))
				img.writeROOTsurf(string(*it).append("-surf.png"));
			if (vm.count("output-zhist"))
				img.writeROOTzhist(string(*it).append("-hist-z.png"));

			images.push_back(std::move(img));
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

