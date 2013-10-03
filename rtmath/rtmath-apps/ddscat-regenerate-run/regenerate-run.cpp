/* This program reads in a ddscat.par file along with a shape (and 
 * possibly a .avg file) and uses these to reconstruct a ddscat-ready 
 * folder.
 *
 * It attempts to align multiple files to match up based on filenames.
 * avg files provide rotations, aeff, wave, dielectrics and scattering angles.
 * shp files provide the actual shape.
 * par files provide a base when writing a new par file.
 */

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <sstream>
#include <map>
#include <vector>
#include <cstdlib>
#include <Ryan_Debug/debug.h>
#include <Ryan_Serialization/serialization.h>

#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
#include "../../rtmath/rtmath/ddscat/ddRunSet.h"
#include "../../rtmath/rtmath/ddscat/dielTabFile.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/error/error.h"

int main(int argc, char** argv)
{
	using namespace std;
	try {
		namespace po = boost::program_options;
		using std::endl;
		using std::cerr;
		using std::string;
		using std::vector;

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("avg,a", po::value<vector<string> >(), "Select avg files")
			("shape,s", po::value<vector<string> >(), "Select shape files")
			("par,p", po::value<string>(), "Select par file")
			("output,o", po::value<string>(), "Select output directory base")
			;

		po::positional_options_description p;
		//p.add("inputs",-1);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);    

		std::map<std::string, rtmath::ddscat::dataset> maps;

		vector<string> avgs, shapes; // May be expanded by os.
		string par, outbase;

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << endl;
			cerr << message << endl;
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");

		if (!vm.count("shape")) doHelp("Need to specify shape files");
		if (!vm.count("avg")) doHelp("Need to specify avg files");
		if (!vm.count("par")) doHelp("Need to specify par file");
		if (!vm.count("output")) doHelp("Need to specify output");

		shapes = vm["shape"].as<vector<string> >();
		avgs = vm["avg"].as<vector<string> >();
		par = vm["par"].as<string>();
		outbase = vm["output"].as<string>();
		
		using namespace boost::filesystem;
		using namespace rtmath::ddscat;

		// Check / create output base directory
		path pOut(outbase);
		if (exists(outbase)) doHelp("Need to select an empty output directory");
		boost::filesystem::create_directory(pOut);

		// No recursion / symlink following here. Everything is treated 
		// as a valid input.
		using rtmath::ddscat::dataset;

		auto matchIDS = [&](const vector<string> &v)
		{
			for (auto &it : v)
			{
				string prefix = dataset::getPrefix(it);
				if (!maps.count(prefix))
					maps[prefix] = dataset(prefix);
				path pfile(it);
				if (!exists(pfile)) continue;
				if (!dataset::isValid(pfile)) continue;
				if (dataset::isAvg(pfile)) maps[prefix].ddres.push_back(pfile);
				if (dataset::isShape(pfile)) maps[prefix].shapefile = pfile;
			}
		};

		/*
		if (avgs.size() == 1)
		{
			// Special case where the exact data is specified
			string prefix = "";

		} else {
			*/
			matchIDS(shapes);
			matchIDS(avgs);
		//}
		rtmath::ddscat::ddPar parFile(par);

		for (auto &d : maps)
		{
			path pa = pOut / path(d.first);
			boost::filesystem::create_directory(pa);
			for (auto &pavg : d.second.ddres)
			{
				ddOutputSingle avg(pavg.string());
				path p = pa / path(pavg).filename();
				boost::filesystem::create_directory(p);
				// Creating shapefile hard links if possible
				try {
					boost::filesystem::create_hard_link(d.second.shapefile, p / path("shape.dat"));
				} catch (std::exception&)
				{
					boost::filesystem::create_symlink(d.second.shapefile, p / path("shape.dat"));
				}
				// Write the diel.tab files
				/// \todo Write a file for each dielectric
				boost::shared_ptr<dielTab> diel = dielTab::generate(avg.getM());
				diel->write( (p / path("diel.tab")).string());
				// Write the ddscat.par file
				ddPar ppar = parFile;
				ppar.setDiels(vector<string>(1,string("diel.tab")));
				ppar.setAeff(avg.aeff(),avg.aeff(),1,"lin");
				ppar.setWavelengths(avg.wave(), avg.wave(), 1, "lin");
				/// \todo Extraction of rotations
				//ppar.setRots(avg.
				/// \todo Extraction of scattering angles
				ppar.writeFile( (p / path("shape.dat")).string() );
			}
		}
	} catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}
