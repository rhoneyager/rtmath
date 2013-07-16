/* This program is designed to manipulate ddscat files. It can set and retreive common ddscat.par variables for use with scripting.
 * It is only designed for ddscat.par files. To modify other files, use a different program */

#pragma warning( push )
#pragma warning( disable : 4996 ) // Dumb boost uuid warning
#pragma warning( disable : 4800 ) // forcing non-bool type to true or false
#pragma warning( disable : 4521 ) // multiple copy constructors in some PCL stuff
#pragma warning( disable : 4244 ) // warning C4244: '=' : conversion from 'double' to 'float', possible loss of data in FLANN
#pragma warning( disable : 4068 ) // unknown gcc pragmas

#include <cmath>
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <algorithm>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>

#include <Ryan_Debug/debug.h>

#pragma warning( pop ) 
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/common_templates.h"
//#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-ddscat-generate-ellipsoid\n\n";

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("write,w", "Insert the ellipsoid information into a ddscat.par file")
			("input,i", po::value< string >(), "input ddscat.par file")
			("output,o", po::value< string >(), "output ddscat.par file (defaults to input)")

			("aeff,a", po::value<double>(), "Set effective radius (um)")
			("dipole-spacing,d", po::value<double>(), "Set interdipole spacing (um)")
			("aspect-ratios", po::value<string>()->default_value("1:1:1"), 
			  "Set aspect ratio relations (used to determine SHPAR1-3).")
			;

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);    

		auto doHelp = [&](const std::string &message)
		{
			cerr << message << "\n";
			cerr << desc << "\n";
			exit(1);
		};
		if (vm.count("help") || argc == 1) doHelp("");

		boost::shared_ptr<rtmath::ddscat::ddPar> par;

		string input;
		if (vm.count("input"))
		{
			input = vm["input"].as<string>();
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
			par = boost::shared_ptr<rtmath::ddscat::ddPar>( new rtmath::ddscat::ddPar(input) );
		} else {
			par = boost::shared_ptr<rtmath::ddscat::ddPar>(new rtmath::ddscat::ddPar );
		}

		string output = input;
		bool doWrite = false;

		if (vm.count("output"))
		{
			output = vm["output"].as<string>();
			doWrite = true;
		}
		if (vm.count("write")) doWrite = true;

		par->setShape("ELLIPSOID");

		if (!vm.count("aeff")) doHelp("Need to specify aeff");
		double aeff = vm["aeff"].as<double>();
		par->setAeff(aeff, aeff, 1, "LIN");

		if (!vm.count("dipole-spacing")) doHelp("Need to specify interdipole spacing");
		double d = vm["dipole-spacing"].as<double>();

		vector<double> ar(3,1.0);
		vector<string> vsar;
		string sar = vm["aspect-ratios"].as<string>();
		boost::split(vsar,sar,boost::is_any_of(":"));
		vsar.resize(3,"1.0");
		auto it = vsar.begin();
		auto ot = ar.begin();
		for (; it != vsar.end(); ++it, ++ot)
		{
			*ot = boost::lexical_cast<double>(*it);
			if (it != vsar.begin())
				*ot /= ar[0]; // Scaling to 1:ab:ac ratios
		}
		ar[0] = 1.0;

		// Calculate shape parameters
		vector<double> shpar(3,1.0);
		// a = 2x/d, b=2y/d, c=2z/d
		// aeff^2 = x^2+y^2+z^2
		// ar[0] = 1, other ars are scaled according to this.
		shpar[0] = (2. * aeff / d) * pow(1./(ar[1]*ar[2]) , 1./3.);
		shpar[1] = (2. * aeff / d) * pow(pow(ar[1],2.)/ar[2], 1./3.);
		shpar[2] = (2. * aeff / d) * pow(pow(ar[2],2.)/ar[1], 1./3.);
				
		par->shpar(0, shpar[0]);
		par->shpar(1, shpar[1]);
		par->shpar(2, shpar[2]);

		cerr << "aeff:\t" << aeff << endl;
		cerr << "d:   \t" << d << endl;
		cerr << "ar 0:\t" << ar[0] << endl;
		cerr << "ar 1:\t" << ar[1] << endl;
		cerr << "ar 2:\t" << ar[2] << endl;
		cerr << "shp1:\t" << shpar[0] << endl;
		cerr << "shp2:\t" << shpar[1] << endl;
		cerr << "shp3:\t" << shpar[2] << endl;

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

