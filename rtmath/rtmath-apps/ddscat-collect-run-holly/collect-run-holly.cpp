#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/math/constants/constants.hpp>
#include <iostream>
#include <sstream>
#include <map>
#include <vector>
#include <cstdlib>
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/splitSet.h>
#include <Ryan_Debug/fs.h>

#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/ddscat/ddOriData.h"
#include "../../rtmath/rtmath/ddscat/ddRunSet.h"
#include "../../rtmath/rtmath/ddscat/dielTabFile.h"
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/error/debug.h"

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
			("frequency,f", po::value<double>(), "Specify frequency (GHz)")
			("match-by-folder", "Instead of matching by standard prefix, match by containing folder")
			("match-by-dipoles", "Instead of matching by a standard prefix, match avg files and shape "
			"files by the number of dipoles. Necessary for Liu raw pristine flake extraction. "
			"Will use a default par file this way.")
			("calc-dipole-extent", po::value<bool>()->default_value(true), "Read the shape and calculate the "
			"dimension sizes. Used to tweak the parameter file.")
			("ice-temp,T", po::value<double>(), "Specify ice temperature. Used if a dielectric is regenerated.")
			("only-one-avg", "Use only one avg file of each id for regeneration.")
			("dipole-spacing,d", po::value<double>(), "Set interdipole spacing")
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
	
		double dSpacing = 0;
		if (vm.count("dipole-spacing")) dSpacing = vm["dipole-spacing"]
			.as<double>();
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

		double temp = 0, freq = 0;
		if (vm.count("ice-temp")) temp = vm["ice-temp"].as<double>();
		if (vm.count("frequency"))
			freq = vm["frequency"].as<double>();
		else doHelp("Need to specify frequency");
		double wave = rtmath::units::conv_spec("GHz", "um").convert(freq);

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
						auto opts = Ryan_Debug::registry::IO_options::generate();
						opts->filename(pfile.string());
						opts->setVal<bool>("headerOnly", true);

						auto s = rtmath::ddscat::shapefile::shapefile::generate();
						s->readMulti(nullptr, opts);
						// OLD: s.readHeaderOnly(pfile.string());
						nDipoles = s->numPoints;
					}
					// If an avg file, read and extract dipole number.
					else if (dataset::isAvg(pfile)) {
						rtmath::ddscat::ddOutput a;
						//a.readFile(pfile.string(), ".avg");
						auto b = rtmath::ddscat::ddOriData::generate(a);
						b->readFile(pfile.string(), ".avg");
						nDipoles = b->numDipoles();
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

			string sParFile;
			auto parFile = rtmath::ddscat::ddPar::generate();
			if (d.second.parfile.empty() && pars.size())
			{
				cerr << "\tUsing generic par file " << pars[0] << endl;
				sParFile = pars[0].string();
				parFile = rtmath::ddscat::ddPar::generate(pars[0].string());
			}
			else if (d.second.parfile.empty() && !pars.size())
				RDthrow(Ryan_Debug::error::xMissingFile())
				<< Ryan_Debug::error::file_name("ddscat.par");
			else {
				sParFile = d.second.parfile.string();
				parFile = rtmath::ddscat::ddPar::generate(d.second.parfile.string());
				cerr << "\tUsing matched par file " << d.second.parfile.string() << endl;
			}

			// Reopen the shape file to determine dipole extent
			vector<size_t> dims(3,0);
			auto s = rtmath::ddscat::shapefile::shapefile::generate(d.second.shapefile.string());
			double aeff_di = pow((float)s->numPoints*3.f
				/ (4.f*boost::math::constants::pi<float>()), 1.f / 3.f);
			double aeff_um = aeff_di * dSpacing;

			if (calcDipoleExtent)
			{
				// Extra padding is needed by ddscat...
				dims[0] = (size_t) (s->maxs(0) - s->mins(0) + 20);
				dims[1] = (size_t) (s->maxs(1) - s->mins(1) + 20);
				dims[2] = (size_t) (s->maxs(2) - s->mins(2) + 20);
				cerr << "\tDipole extent is " << dims[0] << ", " << dims[1] << ", " << dims[2] << endl;
			}
			else cerr << "\tUsing initial dipole extent" << endl;

			auto linkObj = [&](
				const boost::filesystem::path &src,
				const boost::filesystem::path &spath) -> bool
			{
				if (boost::filesystem::exists(spath)) return false;
				try {
					// Creating shapefile hard links if possible
					boost::filesystem::create_hard_link(src, spath);
				}
				catch (std::exception&)
				{
					try {
						// Then try making symlinks
						boost::filesystem::create_symlink(makePathAbsolute(src), spath);
					}
					catch (std::exception &e)
					{
						cerr << "Cannot make link to " << src << endl;
						cerr << "Exception is " << e.what() << endl;
						return false;
						// Then just do direct copying
						//boost::filesystem::copy_file(src, spath));
					}
				}
				return true;
			};
			auto createPar = [&](const boost::filesystem::path &ppath, ddPar &ppar, 
				double aeff)
			{
				ppar.setWavelengths(wave, wave, 1, "LIN");

				if (dielectrics.size()) ppar.setDiels(dielectrics);
				else if (wave && temp) {
					// Regenerate a dielectric based on the refractive index of ice at this frequency/temp combination.
				std::complex<double> m;

				using rtmath::refract::_frequency;
				using rtmath::refract::_temperature;
				using rtmath::refract::_m;
				using rtmath::refract::_provider;
				rtmath::refract::mIce(
					_frequency = freq,
					_temperature = temp,
					_m = m);
				boost::shared_ptr<dielTab> diel = dielTab::generate(m);
				diel->write((ppath.parent_path() / path("diel.tab")).string());
				ppar.setDiels(vector<string>(1, string("diel.tab")));
				} // If no case matches, then the par file skeleton defaults are fine.
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

			for (auto &pavg : d.second.ddres)
			{
				cerr << "Using avg file " << pavg << "\n";
				// Ignore the avg file read. Instead,
				// get the frequency from the command line options,
				// as well as an interdipole spacing. Then generate
				// the ddscat.par file manually.
				//auto a = rtmath::ddscat::ddOutput::generate(
				//	pavg.string(), sParFile, d.second.shapefile.string());
				path p = pa;
				cerr << "\tCreating directory " << p << endl;
				boost::filesystem::create_directory(p);
				if (!linkObj(d.second.shapefile, p / path("shape.dat"))) continue;
				if (!linkObj(pavg, p / path("w000r000.avg"))) continue;
				auto ppar = parFile;

				// Write the diel.tab and ddscat.par files.
				createPar((p / path("ddscat.par")), *(ppar.get()), aeff_um);

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
