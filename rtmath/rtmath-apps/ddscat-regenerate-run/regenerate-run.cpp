/* This program reads in a ddscat.par file along with a shape (and 
 * possibly a .avg file) and uses these to reconstruct a ddscat-ready 
 * folder.
 *
 * It attempts to align multiple files to match up based on filenames.
 * avg files provide rotations, aeff, wave, dielectrics and scattering angles.
 * shp files provide the actual shape.
 * par files provide a base when writing a new par file.
 *
 * If shape files exist in the same filesystem, uses hard links. If on different
 * filesystems, uses symlinks. If a new shape file is generated, only one copy of the
 * file is truly written. Duplicate shape files are matched by hashing.
 *
 * Supports shapefile decimation / enhancement.
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
			("avg,a", po::value<vector<string> >()->multitoken(), "Select avg files")
			("shape,s", po::value<vector<string> >()->multitoken(), "Select shape files")
			("par,p", po::value<vector<string> >(), "Select par file")
			("output,o", po::value<string>(), "Select output directory base")
			("dielectric-files", po::value<vector<string> >(),
			"Force a set of dielectric files to be used. "
			"These will override the standard par file choices.")
			("use-avg-dielectrics",
			"Use the refractive indices that appear in each avg file.")
			("force-frequency", po::value<double>(), "Override frequency (GHz)")
			//("force-intermediate", po::value<bool>(), "Force intermediate output")
			("match-by-folder", "Instead of matching by standard prefix, match by containing folder")
			;

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);

		std::map<std::string, rtmath::ddscat::dataset> maps;

		vector<string> avgs, shapes, pars; // May be expanded by os.
		string outbase;
		bool matchFolder = false;
		if (vm.count("match-by-folder")) matchFolder = true;

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << endl;
			cerr << message << endl;
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");

		if (!vm.count("shape")) doHelp("Need to specify shape files");
		//if (!vm.count("avg")) doHelp("Need to specify avg files");
		if (!vm.count("par")) doHelp("Need to specify par file(s)");
		if (!vm.count("output")) doHelp("Need to specify output");

		shapes = vm["shape"].as<vector<string> >();
		if (vm.count("avg"))
			avgs = vm["avg"].as<vector<string> >();
		pars = vm["par"].as<vector<string> >();
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
		if (!exists(outbase)) //doHelp("Need to select an empty output directory");
			boost::filesystem::create_directory(pOut);

		// No recursion / symlink following here. Everything is treated
		// as a valid input.
		using rtmath::ddscat::dataset;

		auto matchIDS = [&](const vector<string> &v)
		{
			for (auto &it : v)
			{
				string prefix;
				if (!matchFolder)
					prefix = dataset::getPrefix(it);
				else {
					prefix = makePathAbsolute(path(it)).string();
					prefix = path(prefix).remove_filename().string();
				}
				cerr << "p: " << prefix << " init " << it << endl;
				if (!maps.count(prefix))
				{
					maps[prefix] = dataset(prefix);
					cerr << "Adding prefix " << prefix << " from file " << it << endl;
				}
				path pfile(it);
				if (!exists(pfile)) continue;
				if (!dataset::isValid(pfile)) continue;
				if (dataset::isAvg(pfile)) maps[prefix].ddres.push_back(pfile);
				else if (dataset::isShape(pfile)) maps[prefix].shapefile = pfile;
				else if (dataset::isPar(pfile)) maps[prefix].parfile = pfile;
			}
		};

		if (0) // avgs.size() == 1)
		{
			// Special case where the exact data is specified
			string prefix = dataset::getPrefix(path(avgs[0]));
			if (!prefix.size()) prefix = "manual";
			cerr << "Single avg file: " << avgs[0] << endl;
			maps[prefix] = dataset(prefix);
			maps[prefix].ddres.push_back(path(avgs[0]));
			maps[prefix].shapefile = path(shapes.at(0));
		} else {
			matchIDS(shapes);
			matchIDS(avgs);
			if (pars.size() > 1)
				matchIDS(pars);
		}

		for (auto &d : maps)
		{
			path pa = pOut / path(d.first);
			cerr << "Processing " << pa << endl;
			cerr << "\t" << d.second.shapefile << endl;
			if (!boost::filesystem::exists(pa))
				boost::filesystem::create_directory(pa);

			rtmath::ddscat::ddPar parFile;
			if (d.second.parfile.empty() && pars.size())
				parFile = rtmath::ddscat::ddPar(pars[0]);
			else if (d.second.parfile.empty() && !pars.size())
				throw rtmath::debug::xMissingFile("ddscat.par");
			else parFile = rtmath::ddscat::ddPar(d.second.parfile.string());

			auto linkShape = [&](const boost::filesystem::path &spath) -> bool
			{
				try {
					// Creating shapefile hard links if possible
					boost::filesystem::create_hard_link(d.second.shapefile, spath);
				}
				catch (std::exception&)
				{
					try {
						// Then try making symlinks
						boost::filesystem::create_symlink(makePathAbsolute(d.second.shapefile), spath);
					}
					catch (std::exception &e)
					{
						cerr << "Cannot make link to shape file " << d.second.shapefile << endl;
						cerr << "Exception is " << e.what() << endl;
						return false;
						// Then just do direct copying
						//boost::filesystem::copy_file(d.second.shapefile, p / path("shape.dat"));
					}
				}
				return true;
			};
			auto createPar = [&](const boost::filesystem::path &ppath, ddPar &ppar, double aeff, double wave, const std::complex<double> &m)
			{
				if (vm.count("use-avg-dielectrics"))
				{
					if (m.real() == 0)
					{
						//throw rtmath::debug::xBadInput("Missing avg file for dielectric calculation");
						cerr << "Missing avg file for dielectric calculation with " << ppath << endl;
						return;
					}
					/// \todo Allow multiple ms
					boost::shared_ptr<dielTab> diel = dielTab::generate(m);
					diel->write((ppath.parent_path() / path("diel.tab")).string());
					ppar.setDiels(vector<string>(1, string("diel.tab")));
				}
				else {
					if (dielectrics.size())
						ppar.setDiels(dielectrics);
					// If no dieletrics set, then the par file skeleton defaults are fine.
				}
				// If aeff is zero, just stick with the par file definition
				if (aeff)
					ppar.setAeff(aeff, aeff, 1, "LIN");

				if (vm.count("force-frequency"))
				{
					double f = vm["force-frequency"].as<double>();
					double wave = rtmath::units::conv_spec("GHz", "um").convert(f);
					ppar.setWavelengths(wave, wave, 1, "LIN");
				}
				else {
					// If wave is zero, stick with the par file definition
					if (wave)
						ppar.setWavelengths(wave, wave, 1, "LIN");
				}
				// Rotations will match the par file.
				// Scattering angle selection will match the par file.
				ppar.writeFile(ppath.string());
			};


			// If there are no avg files to regenerate from
			if (!d.second.ddres.size())
			{
				path p = pOut / pa.filename();
				//path p = pa / "test";
				cerr << "\tCreating directory " << p << endl;
				boost::filesystem::create_directory(p);
				if (!linkShape(p / path("shape.dat"))) continue;
				ddPar ppar = parFile;
				// Write the diel.tab files
				createPar((p / path("ddscat.par")), ppar, 0, 0, std::complex<double>(0,0));
			}
			// and if there are avg files to regenerate from
			for (auto &pavg : d.second.ddres)
			{
				ddOutputSingle avg(pavg.string());
				path p = pOut / pa.filename(); // / path(pavg).filename();
				cerr << "\tCreating directory " << p << endl;
				boost::filesystem::create_directory(p);
				if (!linkShape(p / path("shape.dat"))) continue;
				ddPar ppar = parFile;
				// Write the diel.tab files
				createPar((p / path("ddscat.par")), ppar, avg.aeff(), avg.wave(), avg.getM());
			}
		}
	} catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}
