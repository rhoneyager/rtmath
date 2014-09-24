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

#include <boost/math/constants/constants.hpp>
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

		rtmath::debug::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("shape,s", po::value<vector<string> >()->multitoken(), "Select shape files")
			("par,p", po::value<string >()->multitoken(), "Select par file")
			("output,o", po::value<string>(), "Select output directory base")
			("dielectric-files", po::value<vector<string> >(),
			"Force a set of dielectric files to be used. "
			"These will override the standard par file choices.")
			("frequency,f", po::value<double>(), "Override frequency (GHz)")
			("temperature,T", po::value<double>(), "Specify ice temperature. Used if a dielectric is regenerated.")

			("dipole-spacing,d", po::value<double>(), "Override the dipole spacing")
			("use-db", po::value<bool>()->default_value(true), "Use database to supplement loaded information")
			("from-db", po::value<bool>()->default_value(false), "Perform search on database and select files matching criteria.")
			("match-hash", po::value<vector<string> >()->multitoken(), "Match lower hashes")
			("match-flake-type", po::value<vector<string> >()->multitoken(), "Match flake types")
			("match-dipole-spacing", po::value<vector<float> >()->multitoken(), "Match typical dipole spacings")
			("match-dipole-numbers", po::value<vector<size_t> >()->multitoken(), "Match typical dipole numbers")
			("match-parent-flake-hash", po::value<vector<string> >()->multitoken(), "Match flakes having a given parent hash")
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

		vector<string> iavgs, ishapes; // May be expanded by os.
		string sparfile;
		string outbase;

		if (!vm.count("par")) doHelp("Need to specify par file(s)");
		if (!vm.count("output")) doHelp("Need to specify output base");

		if (vm.count("shape"))
			ishapes = vm["shape"].as<vector<string> >();

		if (!vm.count("par")) doHelp("Must specify a ddscat.par file for use as a base.");
		sparfile = vm["par"].as<string >();
		outbase = vm["output"].as<string>();

		vector<string> dielectrics;
		if (vm.count("dielectric-files")) dielectrics = vm["dielectric-files"].as<vector<string> >();

		double temp = 0;
		if (vm.count("temperature")) temp = vm["temperature"].as<double>();
		double freq = 0;
		if (vm.count("frequency")) freq = vm["frequency"].as<double>();

		double dSpacing = 0;
		if (vm.count("dipole-spacing"))
			dSpacing = vm["dipole-spacing"].as<double>();

		std::shared_ptr<rtmath::registry::IOhandler> handle, exportHandle;
		std::shared_ptr<rtmath::registry::DBhandler> dHandler;

		/*
		("from-db", "Perform search on database and select files matching criteria.")
		("match-hash", po::value<vector<string> >(), "Match lower hashes")
		("match-flake-type", po::value<vector<string> >(), "Match flake types")
		("match-dipole-spacing", po::value<vector<float> >(), "Match typical dipole spacings")
		("match-parent-flake-hash", po::value<vector<string> >(), "Match flakes having a given parent hash")
		("match-parent-flake", "Select the parent flakes")
		*/
		vector<string> matchHashes, matchFlakeTypes, matchParentHashes;
		rtmath::config::intervals<float> iDipoleSpacing;
		rtmath::config::intervals<size_t> iDipoleNumbers;

		if (vm.count("match-hash")) matchHashes = vm["match-hash"].as<vector<string> >();
		if (vm.count("match-flake-type")) matchFlakeTypes = vm["match-flake-type"].as<vector<string> >();
		if (vm.count("match-parent-flake-hash")) matchParentHashes = vm["match-parent-flake-hash"].as<vector<string> >();
		if (vm.count("match-dipole-spacing")) iDipoleSpacing.append(vm["match-dipole-spacing"].as<vector<string>>());
		if (vm.count("match-dipole-numbers")) iDipoleNumbers.append(vm["match-dipole-numbers"].as<vector<string>>());

		//if (vm.count("match-parent-flake")) matchParentFlakes = true;

		using namespace rtmath::ddscat::shapefile;
		auto collection = shapefile::makeCollection();
		auto query = shapefile::makeQuery();
		query->hashLowers.insert(matchHashes.begin(), matchHashes.end());
		query->flakeTypes.insert(matchFlakeTypes.begin(), matchFlakeTypes.end());
		query->refHashLowers.insert(matchParentHashes.begin(), matchParentHashes.end());
		query->dipoleNumbers = iDipoleNumbers;
		query->dipoleSpacings = iDipoleSpacing;

		bool supplementDb = false;
		supplementDb = vm["use-db"].as<bool>();
		bool fromDb = false;
		fromDb = vm["from-db"].as<bool>();

		auto dbcollection = rtmath::ddscat::shapefile::shapefile::makeCollection();
		auto qExisting = rtmath::ddscat::shapefile::shapefile::makeQuery();


		using namespace boost::filesystem;
		using namespace rtmath::ddscat;

		for (auto it = ishapes.begin(); it != ishapes.end(); ++it)
		{
			cerr << "Processing " << *it << endl;
			path pi(*it);
			if (!exists(pi)) throw rtmath::debug::xMissingFile(it->c_str());
			if (is_directory(pi))
			{
				path pt = pi / "target.out";
				path ps = pi / "shape.dat";
				boost::shared_ptr<rtmath::ddscat::shapefile::shapefile> smain;

				if (exists(ps))
				{
					cerr << " found " << ps << endl;
					smain = rtmath::ddscat::shapefile::shapefile::generate(ps.string());
					collection->insert(smain);
				}
			}
			else {
				auto iopts = rtmath::registry::IO_options::generate();
				iopts->filename(*it);
				try {
					vector<boost::shared_ptr<rtmath::ddscat::shapefile::shapefile> > shapes;
					// Handle not needed as the read context is used only once.
					if (rtmath::ddscat::shapefile::shapefile::canReadMulti(nullptr, iopts))
						rtmath::ddscat::shapefile::shapefile::readVector(nullptr, iopts, shapes, query);
					else {
						boost::shared_ptr<rtmath::ddscat::shapefile::shapefile> s = rtmath::ddscat::shapefile::shapefile::generate(*it);
						shapes.push_back(s);
					}
					for (auto &s : shapes)
						collection->insert(s);
				}
				catch (std::exception &e)
				{
					cerr << e.what() << std::endl;
					continue;
				}
			}
		}
		// Perform the query, and then process the matched shapefiles
		auto res = query->doQuery(collection, fromDb, supplementDb, dHandler);



		// Check / create output base directory
		path pOut(outbase);
		if (!exists(outbase)) //doHelp("Need to select an empty output directory");
			boost::filesystem::create_directory(pOut);


		auto processShape = [&](boost::shared_ptr<rtmath::ddscat::shapefile::shapefile> shp)
		{
			cerr << "  Shape " << shp->hash().lower << endl;
			shp->loadHashLocal(); // Load the shape fully, if it was imported from a database

			if (dSpacing && !shp->standardD) shp->standardD = (float)dSpacing;

			if (shp->standardD == 0) doHelp("Need to set standard dipole spacings for some of these files.");
			path outpath = pOut / path(shp->hash().string());
			if (!boost::filesystem::exists(outpath))
				boost::filesystem::create_directory(outpath);

			// Determine dipole extent and patch par file
			vector<size_t> dims(3, 0);
			// Extra padding is needed by ddscat...
			dims[0] = (size_t)(shp->maxs(0) - shp->mins(0) + 20);
			dims[1] = (size_t)(shp->maxs(1) - shp->mins(1) + 20);
			dims[2] = (size_t)(shp->maxs(2) - shp->mins(2) + 20);
			cerr << "\tDipole extent is " << dims[0] << ", " << dims[1] << ", " << dims[2] << endl;

			// Get effective radius
			double aeff_di = pow((float)shp->numPoints*3.f / (4.f*boost::math::constants::pi<float>()), 1.f / 3.f);
			double aeff_um = aeff_di * shp->standardD;

			// Write par file
			rtmath::ddscat::ddPar parFile;
			parFile = rtmath::ddscat::ddPar(sparfile);

			cerr << "\tEffective radius is " << aeff_um << std::endl;

			auto createPar = [&](const boost::filesystem::path &ppath, ddPar &ppar,
				double aeff)
			{
				std::complex<double> m, mAir(1, 0);
				if (freq)
				{
					double wave = rtmath::units::conv_spec("GHz", "um").convert(freq);
					ppar.setWavelengths(wave, wave, 1, "LIN");
				}

				if (dielectrics.size()) ppar.setDiels(dielectrics);
				else if (freq && temp) {
					// Regenerate a dielectric based on the refractive index of ice at this frequency/temp combination.
					rtmath::refract::mIce(freq, temp, m);
					boost::shared_ptr<dielTab> diel = dielTab::generate(m);
					diel->write((ppath.parent_path() / path("diel.tab")).string());
					ppar.setDiels(vector<string>(1, string("diel.tab")));

					if (shp->Dielectrics.count(2))
					{
						if (shp->tags.count("inner-perturbation-numInnerLatticeSites") &&
							shp->tags.count("inner-perturbation-numOccupiedInnerLatticeSites"))
						{
							double numInner = 0, numOccupied = 0, frac = 0;
							numInner = boost::lexical_cast<double>(shp->tags.at("inner-perturbation-numInnerLatticeSites"));
							numOccupied = boost::lexical_cast<double>(shp->tags.at("inner-perturbation-numOccupiedInnerLatticeSites"));
							if (numInner && numOccupied)
							{
								frac = numOccupied / numInner;
							}
							std::complex<double> mEff;
							rtmath::refract::maxwellGarnettEllipsoids(m, mAir, frac, mEff);
							boost::shared_ptr<dielTab> dielEff = dielTab::generate(mEff);
							dielEff->write((ppath.parent_path() / path("dielEff.tab")).string());
							vector<string> diels;
							diels.push_back("diel.tab");
							diels.push_back("dielEff.tab");
							ppar.setDiels(diels);
						}
					}
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
					ppar.insertKey(ddParParsers::DIMENSION, boost::static_pointer_cast< ddParParsers::ddParLine >(line));
				}

				// Rotations will match the par file.
				// Scattering angle selection will match the par file.
				ppar.writeFile(ppath.string());
			};
			createPar(path(outpath / "ddscat.par"), parFile, aeff_um);

			// First dielectric is written by createPar (if not specified on command-line)

			// Write Shape
			shp->writeFile(path(outpath / "shape.dat").string());

		};

		for (const auto &s : *(res.first))
			processShape(rtmath::ddscat::shapefile::shapefile::generate(s));


	}
	catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}
