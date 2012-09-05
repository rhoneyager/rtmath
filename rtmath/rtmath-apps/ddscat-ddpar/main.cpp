/* This program is designed to manipulate ddscat files. It can set and retreive common ddscat.par variables for use with scripting.
 * It is only designed for ddscat.par files. To modify other files, use a different program */

#pragma warning( push )
#pragma warning( disable : 4996 ) // Dumb boost uuid warning
#pragma warning( disable : 4800 ) // forcing non-bool type to true or false
#pragma warning( disable : 4521 ) // multiple copy constructors in some PCL stuff
#pragma warning( disable : 4244 ) // warning C4244: '=' : conversion from 'double' to 'float', possible loss of data in FLANN
#pragma warning( disable : 4068 ) // unknown gcc pragmas

#include "../../rtmath/rtmath/ROOTlink.h"
#include "../../rtmath/rtmath/VTKlink.h"
#include "../../rtmath/rtmath/Garrett/pclstuff.h"
#include "../../rtmath/rtmath/MagickLINK.h"

#include <cmath>
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <algorithm>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/shared_ptr.hpp>

#pragma warning( pop ) 
#include "../../rtmath/rtmath/matrixop.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/error/error.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/matrixop.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/error/debug.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-ddscat-ddpar\n\n";
		rtmath::debug::appEntry(argc, argv);

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("input,i", po::value< string >(), "input ddscat.par file")
			("output,o", po::value< string >(), "output ddscat.par file (defaults to input)")

			("set-version,v", po::value<size_t>(), "Specify version (70,72)")
			("set-frequency,f", po::value<double>(), "Specify frequency (GHz)")
			("set-aeff,a", po::value<double>(), "Set effective radius (um)")
			("set-shape", po::value<string>(), "Set SHAPE parameter (FROM_FILE, ...)")

			("get-version", "Get version (70, 72)")
			("get-frequency", "Get frequency (GHz)")
			("get-aeff", "Get effective radius (um)")
			("get-shape", "Get SHAPE parameter (FROM_FILE, ...)");

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);    

		if (vm.count("help") || argc == 1) {
			cerr << desc << "\n";
			return 1;
		}

		string input = vm["input"].as< string >();
		string output = input;
		if (vm.count("input"))
		{
			// Validate input file
			path pi(input);
			if (!exists(pi)) throw rtmath::debug::xMissingFile(input.c_str());
			if (is_directory(pi))
			{
				path pt = pi / "ddscat.par";
				if (exists(pt)) input = pt.string();
				else throw rtmath::debug::xPathExistsWrongType(input.c_str());
			}
			cerr << "Input par file is: " << input << endl;
		} else {
			cerr << "Need to specify input file.\n" << desc << endl;
			return 1;
		}

		bool doWrite = false;


		// Load the shape file or shape stats file
		cerr << "Processing " << input << endl;
		boost::shared_ptr<rtmath::ddscat::ddPar> par
			( new rtmath::ddscat::ddPar(input) );

		if (vm.count("get-version"))
		{
			cout << "Version: " << par->version() << endl;
		}
		if (vm.count("get-frequency"))
		{
			double min, max;
			size_t n;
			string spacing;
			par->getWavelengths(min,max,n,spacing);
			rtmath::units::conv_spec cnv("um","GHz");
			cout << "Frequency: " << cnv.convert(min) << endl;
		}
		if (vm.count("get-aeff"))
		{
			double min, max;
			size_t n;
			string spacing;
			par->getAeff(min,max,n,spacing);
			cout << "aeff: " << min << endl;
		}
		if (vm.count("get-shape"))
		{
			string shape;
			par->getShape(shape);
			cout << "Shape: " << shape << endl;
		}
		if (vm.count("set-version"))
		{
			doWrite = true;
			par->version(vm["set-version"].as<size_t>());
		}
		if (vm.count("set-frequency"))
		{
			doWrite = true;
			double freq = vm["set-frequency"].as<double>();
			rtmath::units::conv_spec cnv("GHz","um");
			double wvlen = cnv.convert(freq);
			par->setWavelengths(wvlen,wvlen,1,"LIN");
		}
		if (vm.count("set-aeff"))
		{
			doWrite = true;
			double aeff = vm["set-aeff"].as<double>();
			par->setAeff(aeff,aeff,1,"LIN");
		}
		if (vm.count("set-shape"))
		{
			doWrite = true;
			string shape = vm["set-shape"].as<string>();
			par->setShape(shape);
		}

		if (vm.count("output")) output = vm["output"].as< string >();
		if (doWrite)
		{
			cerr << "Output par file is: " << output << endl;
			par->writeFile(output);
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

