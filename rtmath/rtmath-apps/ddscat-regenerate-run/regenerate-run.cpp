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
//#include <Ryan_Serialization/serialization.h>

#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
#include "../../rtmath/rtmath/ddscat/ddRunSet.h"
#include "../../rtmath/rtmath/ddscat/dielTabFile.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

int main(int argc, char** argv)
{
	using namespace std;
	try {
		cerr << "rtmath-ddscat-regenerate-run\n\n";

		namespace po = boost::program_options;
		using std::endl;
		using std::cerr;
		using std::string;
		using std::vector;

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");

		rtmath::debug::add_options(cmdline,config,hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("avg,a", po::value<vector<string> >(), "Select avg files")
			("shape,s", po::value<vector<string> >(), "Select shape files")
			("par,p", po::value<string>(), "Select par file")
			("output,o", po::value<string>(), "Select output directory base")
			("dielectric-files", po::value<vector<string> >(),
			"Force a set of dielectric files to be used. "
			"These will override the standard par file choices.")
			("use-avg-dielectrics",
			"Use the refractive indices that appear in each avg file.")
			;

		po::positional_options_description p;
		//p.add("inputs",-1);

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);

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

		auto makePathAbsolute = [](const boost::filesystem::path &p) -> boost::filesystem::path
		{
			using namespace boost::filesystem;
			if (p.is_absolute()) return p;
			path pp = boost::filesystem::absolute(p);
			return pp;
		};

		vector<string> dielectrics;
		if (vm.count("dielectric-files"))
		{
			dielectrics = vm["dielectric-files"].as<vector<string> >();
			for (auto &d : dielectrics)
			{
				boost::filesystem::path p(d);
				p = makePathAbsolute(p);
				d = p.string();
			}
		}
		
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

		if (avgs.size() == 1)
		{
			// Special case where the exact data is specified
			string prefix = dataset::getPrefix(path(avgs[0]));
			if (!prefix.size()) prefix = "manual";
			maps[prefix] = dataset(prefix);
			maps[prefix].ddres.push_back(path(avgs[0]));
			maps[prefix].shapefile = path(shapes.at(0));
		} else {
			matchIDS(shapes);
			matchIDS(avgs);
		}
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
				try {
					// Creating shapefile hard links if possible
					boost::filesystem::create_hard_link(d.second.shapefile, p / path("shape.dat"));
				} catch (std::exception&)
				{
					try {
						// Then try making symlinks
						boost::filesystem::create_symlink( makePathAbsolute(d.second.shapefile), p / path("shape.dat"));
					} catch (std::exception&)
					{
						// Then just do direct copying
						boost::filesystem::copy_file(d.second.shapefile, p / path("shape.dat"));
					}
				}
				ddPar ppar = parFile;
				// Write the diel.tab files
				if (vm.count("use-avg-dielectrics"))
				{
					/// \todo Allow multiple ms
					boost::shared_ptr<dielTab> diel = dielTab::generate(avg.getM());
					diel->write( (p / path("diel.tab")).string());
					ppar.setDiels(vector<string>(1,string("diel.tab")));
				} else {
					if (dielectrics.size())
						ppar.setDiels(dielectrics);
					// If no dieletrics set, then the par file skeleton defaults are fine.
				}
				
				ppar.setAeff(avg.aeff(),avg.aeff(),1,"lin");
				ppar.setWavelengths(avg.wave(), avg.wave(), 1, "lin");
				// Rotations will match the par file.
				// Scattering angle selection witll match the par file.
				ppar.writeFile( (p / path("ddscat.par")).string() );
			}
		}
	} catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}
