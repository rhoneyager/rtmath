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
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/ddscat/ddOriData.h"
#include "../../rtmath/rtmath/ddscat/ddRunSet.h"
#include "../../rtmath/rtmath/ddscat/dielTabFile.h"
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

enum matchCriteria
{
	MATCH_STANDARD,
	MATCH_FOLDERS,
	MATCH_DIPOLES
};

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
			("par,p", po::value<vector<string> >()->multitoken(), "Select par file")
			("output,o", po::value<string>(), "Select output directory base")
			("dielectric-files", po::value<vector<string> >(),
			"Force a set of dielectric files to be used. "
			"These will override the standard par file choices.")
			("use-avg-dielectrics",
			"Use the refractive indices that appear in each avg file.")
			("force-frequency", po::value<double>(), "Override frequency (GHz)")
			//("force-intermediate", po::value<bool>(), "Force intermediate output")
			("match-by-folder", "Instead of matching by standard prefix, match by containing folder")
			("match-by-dipoles", "Instead of matching by a standard prefix, match avg files and shape "
			"files by the number of dipoles. Necessary for Liu raw pristine flake extraction. "
			"Will use a default par file this way.")
			("calc-dipole-extent", po::value<bool>()->default_value(true), "Read the shape and calculate the "
			"dimension sizes. Used to tweak the parameter file.")
			("ice-temp,T", po::value<double>(), "Specify ice temperature. Used if a dielectric is regenerated.")
			("only-one-avg", "Use only one avg file of each id for regeneration.")
			;

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << endl;
			cerr << message << endl;
			exit(1);
		};
		
		if (vm.count("help") || argc == 1) doHelp("");

		std::map<std::string, rtmath::ddscat::dataset> maps;

		vector<string> iavgs, ishapes, ipars; // May be expanded by os.
		vector<boost::filesystem::path> avgs, shapes, pars;
		string outbase;
		bool calcDipoleExtent = vm["calc-dipole-extent"].as<bool>();
		matchCriteria mc = MATCH_STANDARD;
		if (vm.count("match-by-folder")) mc = MATCH_FOLDERS;
		if (vm.count("match-by-dipoles")) mc = MATCH_DIPOLES;
		
		bool avgOne = false;
		if (vm.count("only-one-avg")) avgOne = true;

		if (!vm.count("shape")) doHelp("Need to specify shape files");
		//if (!vm.count("avg")) doHelp("Need to specify avg files");
		if (!vm.count("par")) doHelp("Need to specify par file(s)");
		if (!vm.count("output")) doHelp("Need to specify output");

		ishapes = vm["shape"].as<vector<string> >();
		if (vm.count("avg"))
			iavgs = vm["avg"].as<vector<string> >();
		ipars = vm["par"].as<vector<string> >();
		outbase = vm["output"].as<string>();

		double temp = 0;
		if (vm.count("ice-temp")) temp = vm["ice-temp"].as<double>();

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

		auto matchIDS = [&](const vector<path> &v)
		{
			for (auto &it : v)
			{
				string prefix;
				path pfile(it);
				if (mc == MATCH_STANDARD)
					prefix = dataset::getPrefix(it);
				else if (mc == MATCH_FOLDERS) {
					prefix = makePathAbsolute(path(it)).string();
					prefix = path(prefix).remove_filename().string();
				} else if (mc == MATCH_DIPOLES) {
					// If a shape file, read shape and extract dipole number
					size_t nDipoles = 0;
					if (dataset::isShape(pfile)) {
						rtmath::ddscat::shapefile::shapefile s;
						s.readHeaderOnly(pfile.string());
						nDipoles = s.numPoints;
					}
					// If an avg file, read and extract dipole number.
					else if (dataset::isAvg(pfile)) {
						rtmath::ddscat::ddOutput a;
						//a.readFile(pfile.string(), ".avg");
						rtmath::ddscat::ddOriData b(a);
						b.readFile(pfile.string(), ".avg");
						nDipoles = b.numDipoles();
					}
					if (!nDipoles) continue; // Prefix 0 is not valid. Wrong file type.
					std::ostringstream ntos;
					ntos << nDipoles;
					prefix = ntos.str();
					//prefix = makePathAbsolute(path(it)).string();
					//prefix = path(prefix).remove_filename().string();
				}
				cerr << "prefix: " << prefix << " for file " << it << endl;
				if (!maps.count(prefix))
				{
					maps[prefix] = dataset(prefix);
					cerr << "Adding prefix " << prefix << " from file " << it << endl;
				}
				if (!exists(pfile)) continue;
				//if (!dataset::isValid(pfile)) continue;
				if (dataset::isAvg(pfile)) {
					maps[prefix].ddres.push_back(pfile);
					cerr << "\tMatched as avg file\n";
				} else if (dataset::isPar(pfile) || 
						pfile.string().find("par") != std::string::npos) {
					maps[prefix].parfile = pfile;
					cerr << "\tMatched as par file\n";
				} else if (!dataset::isValid(pfile)) continue;
				else if (dataset::isShape(pfile)) {
					// TODO: shape file match can catch par files. It is 
					// too loose.
					maps[prefix].shapefile = pfile;
					cerr << "\tMatched as shape file\n";
				}
			}
		};

		auto expandFolders = [&](const vector<string> &src, vector<path> &dest)
		{
			dest.clear();
			for (auto s : src)
			{
				using namespace boost::filesystem;
				path p(s);
				if (is_directory(p))
					copy(directory_iterator(p), 
					directory_iterator(), back_inserter(dest));
				else dest.push_back(p);
			}
		};

		/*
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
			*/
		expandFolders(ishapes, shapes);
		expandFolders(iavgs, avgs);
		expandFolders(ipars, pars);

		matchIDS(shapes);
		matchIDS(avgs);
		if (pars.size() > 1)
			matchIDS(pars);
		//}

		for (auto &d : maps)
		{
			path pa = pOut / (path(d.first).filename());
			cerr << "Processing " << pa << endl;
			cerr << "\tUsing shape file " << d.second.shapefile << endl;
			if (!boost::filesystem::exists(pa))
				boost::filesystem::create_directory(pa);

			rtmath::ddscat::ddPar parFile;
			if (d.second.parfile.empty() && pars.size())
			{
				parFile = rtmath::ddscat::ddPar(pars[0].string());
				cerr << "\tUsing generic par file " << pars[0] << endl;
			}
			else if (d.second.parfile.empty() && !pars.size())
				throw rtmath::debug::xMissingFile("ddscat.par");
			else {
				parFile = rtmath::ddscat::ddPar(d.second.parfile.string());
				cerr << "\tUsing matched par file " << d.second.parfile.string() << endl;
			}

			// Reopen the shape file to determine dipole extent
			vector<size_t> dims(3,0);
			if (calcDipoleExtent)
			{
				rtmath::ddscat::shapefile::shapefile s;
				s.read(d.second.shapefile.string());
				// Extra padding is needed by ddscat...
				dims[0] = (size_t) (s.maxs(0) - s.mins(0) + 20);
				dims[1] = (size_t) (s.maxs(1) - s.mins(1) + 20);
				dims[2] = (size_t) (s.maxs(2) - s.mins(2) + 20);
				cerr << "\tDipole extent is " << dims[0] << ", " << dims[1] << ", " << dims[2] << endl;
			}
			else cerr << "\tUsing initial dipole extent" << endl;

			auto linkShape = [&](const boost::filesystem::path &spath) -> bool
			{
				if (boost::filesystem::exists(spath)) return false;
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
			auto createPar = [&](const boost::filesystem::path &ppath, ddPar &ppar, 
				double aeff, double wave, std::complex<double> m)
			{
				if (vm.count("force-frequency"))
				{
					double f = vm["force-frequency"].as<double>();
					wave = rtmath::units::conv_spec("GHz", "um").convert(f);
					ppar.setWavelengths(wave, wave, 1, "LIN");
				} else {
					// If wave is zero, stick with the par file definition
					if (wave)
						ppar.setWavelengths(wave, wave, 1, "LIN");
				}

				if (dielectrics.size()) ppar.setDiels(dielectrics);
				else if (vm.count("use-avg-dielectrics"))
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
				else if (wave && temp) {
					// Regenerate a dielectric based on the refractive index of ice at this frequency/temp combination.
					double f = rtmath::units::conv_spec("um", "GHz").convert(wave);
					rtmath::refract::mIce(f, temp, m);
					boost::shared_ptr<dielTab> diel = dielTab::generate(m);
					diel->write((ppath.parent_path() / path("diel.tab")).string());
					ppar.setDiels(vector<string>(1, string("diel.tab")));
				} // If no case matches, then the par file skeleton defaults are fine.
				
				// If aeff is zero, just stick with the par file definition
				if (aeff)
					ppar.setAeff(aeff, aeff, 1, "LIN");

				// Set initial memory dimensions
				if (dims[0])
				{
					// Using the older interface, just because it is convenient
					boost::shared_ptr< ddParParsers::ddParLineSimplePlural<size_t> > line
						(new ddParParsers::ddParLineSimplePlural<size_t>(ddParParsers::DIMENSION));
					line->set(dims);
					ppar.insertKey(ddParParsers::DIMENSION,boost::static_pointer_cast< ddParParsers::ddParLine >(line));
				}

				// Rotations will match the par file.
				// Scattering angle selection will match the par file.
				ppar.writeFile(ppath.string());
			};


			// If there are no avg files to regenerate from
			if (!d.second.ddres.size())
			{
				cerr << "\tNot using avg file\n";
				path p = pa; //pOut / pa.filename();
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
				cerr << "Using avg file " << pavg << "\n";
				//ddOriData avg(d.second.ddres)

				rtmath::ddscat::ddOutput a;
				//a.readFile(pfile.string(), ".avg");
				rtmath::ddscat::ddOriData avg(a);
				avg.readFile(pavg.string(), ".avg");

				//ddOutputSingle avg(pavg.string(),".avg");
				path p = pa;
				// path p = pOut / pa.filename(); // / path(pavg).filename();
				cerr << "\tCreating directory " << p << endl;
				boost::filesystem::create_directory(p);
				if (!linkShape(p / path("shape.dat"))) continue;
				ddPar ppar = parFile;
				// Write the diel.tab files
				createPar((p / path("ddscat.par")), ppar, avg.aeff(), avg.wave(), avg.M());

				if (avgOne) break;
			}
		}
	} catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}
