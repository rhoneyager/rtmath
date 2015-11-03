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
#include "../../rtmath/rtmath/defs.h"
#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/shared_ptr.hpp>

#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/hash.h>
#include <Ryan_Debug/error.h>
#include <Ryan_Debug/config.h>
#include <Ryan_Debug/splitSet.h>

#pragma warning( pop )
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/units.h"
//#include "../../rtmath/rtmath/io.h"
#include "../../rtmath/rtmath/error/debug.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-ddscat-ddpar\n\n";

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		rtmath::debug::add_options(cmdline, config, hidden);
		ddscat::ddPar::add_options(cmdline, config, hidden);
		//Ryan_Serialization::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value< string >(), "input ddscat.par file")
			("output,o", po::value< string >(), "output ddscat.par file (defaults to input)")
			("inputshape,s", po::value<string>(), "input shapefile (for patching)")
			("dipole-spacing,d", po::value<double>(), "Force assumed dipole spacing (for use when setting aeff from shape)")

			("set-version,v", po::value<size_t>(), "Specify version (70, 72, ...)")
			("set-frequency,f", po::value<double>(), "Specify frequency (GHz)")
			("set-aeff,a", po::value<double>(), "Set effective radius (um)")
			("set-shape", po::value<string>(), "Set SHAPE parameter (FROM_FILE, ...)")
			("set-shapeparams", po::value<vector<double> >(), "Set shape params (double, double, double)")
			("set-intermediate-output", po::value<bool>(), "Set IWRKSC")
			("set-diels,D", po::value<vector<std::string> >()->multitoken(), "Set dielectric files. Can specify multiple times for "
			 "multiple refractive indices. In this case, the dielectrics follow command-line ordering.")
			("set-betas", po::value<string>(), "Set beta rotations. Give interval notation for first, number and last.")
			("set-thetas", po::value<string>(), "Set theta rotations. See set-betas.")
			("set-phis", po::value<string>(), "Set phi rotations. See set-betas.")

			("get-version", "Get version (70, 72, ...)")
			("get-frequency", "Get frequency (GHz)")
			("get-aeff", "Get effective radius (um)")
			("get-shape", "Get SHAPE parameter (FROM_FILE, ...)")
			("get-shapeparams", "Get shape parameters")
			("get-intermediate-output", "Get IWRKSC")
			("get-diels", "Get all dielectrics")
			("get-betas", "Get betas")
			("get-thetas", "Get thetas")
			("get-phis", "Get phis")
			;

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);  

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << "\n";
			if (message.size())
				cerr << message << "\n";
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");

		rtmath::debug::process_static_options(vm);
		//Ryan_Serialization::process_static_options(vm);
		ddscat::ddPar::process_static_options(vm);

		if (!vm.count("input")) doHelp("Need to specify input file.\n");

		string input = vm["input"].as< string >();
		string output = input;
		// Validate input file
		path pi(input);
		if (!exists(pi)) RDthrow(Ryan_Debug::error::xMissingFile())
			<< Ryan_Debug::error::file_name(input);
		if (is_directory(pi))
		{
			path pt = pi / "ddscat.par";
			if (exists(pt)) input = pt.string();
			else RDthrow(Ryan_Debug::error::xPathExistsWrongType())
				<< Ryan_Debug::error::file_name(input);
		}
		cerr << "Input par file is: " << input << endl;

		bool doWrite = false;


		// Load the shape file or shape stats file
		cerr << "Processing " << input << endl;
		auto par = rtmath::ddscat::ddPar::generate(input);

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
		rtmath::ddscat::rotations rot(*(par.get()));
		string nb, nt, np;
		rot.betas(nb);
		rot.thetas(nt);
		rot.phis(np);
		if (vm.count("get-betas")) {
			cout << "Betas: " << nb << endl;
		}
		if (vm.count("get-thetas")) {
			cout << "Thetas: " << nt << endl;
		}
		if (vm.count("get-phis")) {
			cout << "Phis: " << np << endl;
		}
		if (vm.count("get-shape"))
		{
			string shape;
			par->getShape(shape);
			cout << "Shape: " << shape << endl;
		}
		if (vm.count("get-shapeparams"))
		{
			cout << "Shapeparams: " << par->shpar(0) << " " << 
				par->shpar(1) << " " << par->shpar(2) << endl;
		}
		if (vm.count("get-intermediate-output"))
		{
			bool iwrksc = par->writeSca();
			cout << "iwrksc: " << iwrksc << endl;
		}
		if (vm.count("get-diels"))
		{
			cout << "diels:" << endl;
			vector<string> dielFiles;
			par->getDiels(dielFiles);
			for (auto &file : dielFiles)
			{
				cout << "\t" << file << endl;
			}
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
		if (vm.count("set-betas")) {
			nb = vm["set-betas"].as<string>();
		}
		if (vm.count("set-thetas")) {
			nt = vm["set-thetas"].as<string>();
		}
		if (vm.count("set-phis")) {
			np = vm["set-phis"].as<string>();
		}
		if (vm.count("set-betas") || vm.count("set-thetas") || vm.count("set-phis")) {
			double bB, bT, bP, eB, eT, eP, junkD;
			size_t sB, sT, sP;
			std::string junk;
			Ryan_Debug::splitSet::extractInterval<double>(nb, bB, eB, junkD, sB, junk);
			Ryan_Debug::splitSet::extractInterval<double>(nt, bT, eT, junkD, sT, junk);
			Ryan_Debug::splitSet::extractInterval<double>(np, bP, eP, junkD, sP, junk);
			auto rotnew = rtmath::ddscat::rotations::create
				(bB, eB, sB, bT, eT, sT, bP, eP, sP);
			par->setRots(*(rotnew.get()));
		}
		if (vm.count("set-shape"))
		{
			doWrite = true;
			string shape = vm["set-shape"].as<string>();
			par->setShape(shape);
		}
		if (vm.count("set-shapeparams"))
		{
			doWrite = true;
			vector<double> s = vm["set-shapeparams"].as<vector<double> >();
			if (s.size() != 3) RDthrow(Ryan_Debug::error::xBadInput())
				<< Ryan_Debug::error::otherErrorText("set-shapeparams needs three doubles as input");
			par->shpar(0, s[0]);
			par->shpar(1, s[1]);
			par->shpar(2, s[2]);
		}
		if (vm.count("set-intermediate-output"))
		{
			doWrite = true;
			bool iwrksc = vm["set-intermediate-output"].as<bool>();
			par->writeSca(iwrksc);
		}
		if (vm.count("set-diels"))
		{
			doWrite = true;
			vector<string> dielFiles;
			dielFiles = vm["set-diels"].as<vector<string> >();
			par->setDiels(dielFiles);
		}
		if (vm.count("inputshape"))
		{
			doWrite = true;
			string shpfile = vm["inputshape"].as<string>();

			std::vector<boost::shared_ptr<rtmath::ddscat::shapefile::shapefile> > shps;
			Ryan_Debug::io::readObjs(shps, shpfile);
			if (!shps.size()) doHelp("File must contain shapes");
			auto shp = shps[0];

			double dSpacing = shp->standardD;
			double numPoints = (double)shp->numPoints;
			if (vm.count("dipole-spacing")) dSpacing = vm["dipole-spacing"].as<double>();
			if (!dSpacing) doHelp("Need to specify interdipole spacing (um) for this shape.");
			double aeff_dipoles = pow(3.0 * numPoints * shp->d.prod() / (4.0 * boost::math::constants::pi<double>()), 1. / 3.);

			double aeff_um = aeff_dipoles * dSpacing;
			par->setAeff(aeff_um, aeff_um, 1, "LIN");
		}

		if (vm.count("output")) output = vm["output"].as< string >();
		if (doWrite || vm.count("output"))
		{
			cerr << "Output par file is: " << output << endl;
			par->writeFile(output);
		}
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

