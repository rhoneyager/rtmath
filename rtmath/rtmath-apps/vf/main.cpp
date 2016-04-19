#include <iostream>
#include "../../rtmath/rtmath/defs.h"
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/math/constants/constants.hpp>
#include <fstream>
#include <complex>
#include <set>
#include <vector>
#include <sstream>
#include <string>
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/splitSet.h>
#include <Ryan_Debug/error.h>
#include <Ryan_Debug/logging.h>
#include <boost/log/sources/global_logger_storage.hpp>
#include "../../rtmath/rtmath/density/density.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/conversions/convertLength.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/points.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/registry.h"
#include "../../rtmath/rtmath/error/debug.h"

BOOST_LOG_INLINE_GLOBAL_LOGGER_CTOR_ARGS(
	m_app,
	boost::log::sources::severity_channel_logger_mt< >,
	(boost::log::keywords::severity = Ryan_Debug::log::error)
	(boost::log::keywords::channel = "app"));

#undef FL
#undef mylog
#define FL "main.cpp, line " << (int)__LINE__ << ": "
#define mylog(x) { std::ostringstream l; l << FL << x; BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_1) << l.str(); }

int main(int argc, char *argv[])
{
	using namespace std;
	using namespace rtmath::units::keywords;
	using rtmath::units::convertLength;
	try {
		cerr << "rtmath-vf" << endl;
		const double pi = boost::math::constants::pi<double>();
		// Do processing of argv
		namespace po = boost::program_options;
		po::options_description desc("Allowed options"), cmdline("Command-line options"),
			runmatch("Run-matching options"),
			config("Config options"), hidden("Hidden options"), oall("all options");

		rtmath::debug::add_options(cmdline, config, hidden);
		rtmath::ddscat::stats::shapeFileStats::add_options(cmdline, config, hidden);

		runmatch.add_options()
			("dipole-spacing,d", po::value<double>(), "Set dipole spacing for file exports.")
			("input-shape,i", po::value< vector<string> >(), "Input shape files")
			("use-db", po::value<bool>()->default_value(true), "Use database to supplement loaded information")
			("from-db", po::value<bool>()->default_value(false), "Perform search on database and select files matching criteria.")
			("match-hash", po::value<vector<string> >()->multitoken(), "Match lower hashes")
			("match-flake-type", po::value<vector<string> >()->multitoken(), "Match flake types")
			("match-dipole-spacing", po::value<vector<float> >()->multitoken(), "Match typical dipole spacings")
			("match-dipole-numbers", po::value<vector<size_t> >()->multitoken(), "Match typical dipole numbers")
			("match-parent-flake-hash", po::value<vector<string> >()->multitoken(), "Match flakes having a given parent hash")
			("match-parent-flake", "Select the parent flakes")

			("convolution-radius", po::value<double>()->default_value(3),
			 "Sets the radius for the convolution operation")
			("convolution-threshold", po::value<double>()->default_value(0.1),
			 "Sets the threshold volume fraction for the convolution valid threshold.")
			("voronoi-depth", po::value<size_t>()->default_value(3), "Sets the internal voronoi depth for scaling")
			("voronoi-offset-factor", po::value<double>()->default_value(3), "Sets the factor used in the Voronoi offset "
			 "method")
			("ar-method", po::value<string>()->default_value("Max_Ellipsoids"),
			"Max_Ellipsoids: Force aspect ratios to be ellipsoids, following the max AR calculated in stats code. "
			"Spheres: Force aspect ratios to be spheres, instead of stats-determined spheroids. "
			"(TODO SA_V_Ellipsoids: Select AR of ellipsoid with matching surface area to volume ratio).")
			("tag", po::value<string>(), "A tag field that is added to keep "
			 "track of different runs.")
			;

		cmdline.add_options()
			("help,h", "produce help message")
			("output-prefix,o", po::value<string>()->default_value("output"), "Set output file (required)")
			;

		desc.add(cmdline).add(config).add(runmatch);
		oall.add(cmdline).add(runmatch).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);
		rtmath::ddscat::stats::shapeFileStats::process_static_options(vm);

		auto& lg = m_app::get();
		// Begin checking parameters
		auto doHelp = [&](const std::string& s)
		{
			cout << s << endl;
			cout << desc << endl;
			exit(3);
		};

		if (vm.count("help") || vm.size() == 0) doHelp("");

		string oprefix;
		if (vm.count("output-prefix"))
			oprefix = vm["output-prefix"].as<string>();

		double dSpacing = 0;
		if (vm.count("dipole-spacing"))
			dSpacing = vm["dipole-spacing"].as<double>();

		string tag;
		if (vm.count("tag")) tag = vm["tag"].as<string>();
		vector<string> inputs;
		if (vm.count("input-shape"))
		{
			inputs = vm["input-shape"].as< vector<string> >();
			cerr << "Input files are:" << endl;
			for (auto it = inputs.begin(); it != inputs.end(); it++)
				cerr << "\t" << *it << "\n";
		};
		std::shared_ptr<Ryan_Debug::registry::DBhandler> dHandler;
		vector<string> matchHashes, matchFlakeTypes, matchParentHashes;
		Ryan_Debug::splitSet::intervals<float> iDipoleSpacing;
		Ryan_Debug::splitSet::intervals<size_t> iDipoleNumbers;
		//bool matchParentFlakes;

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

		using namespace boost::filesystem;
		auto dbcollection = rtmath::ddscat::shapefile::shapefile::makeCollection();
		auto qExisting = rtmath::ddscat::shapefile::shapefile::makeQuery();
		// Load in all local shapefiles, then perform the matching query
		vector<string> vinputs;
		for (auto it = inputs.begin(); it != inputs.end(); it++)
		{
			cerr << "Processing " << *it << endl;
			path pi(*it);
			if (!exists(pi)) RDthrow(Ryan_Debug::error::xMissingFile())
				<< Ryan_Debug::error::file_name(*it);
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
				auto iopts = Ryan_Debug::registry::IO_options::generate();
				iopts->filename(*it);
				try {
					vector<boost::shared_ptr<rtmath::ddscat::shapefile::shapefile> > shapes;
					Ryan_Debug::io::readObjs(shapes, *it);
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



		// Need shape type - can get from pre-construct flaketype
		// Max dimension - dipole spacing and shape stats
		// Effective radius
		// Voronoi interior volume fraction
		// Percent of total mass classified as the interior
		// Honeyager 2016 volume fraction ( the guess )
		// Whole convoluted volume fraction (C=3)
		// Convoluted volume fraction in interior only
		// Convoluted volume fraction in surface only
		struct flakedata {
			string hash;
			double aeff_um, md_mm, vol_mm3,
				vf_cs, vf_ce,
				vf_vint, vf_hon,
				vf_ctot, vf_cint,
				vf_cext, intmfrac, ar;
		};

		size_t int_voro_depth = vm["voronoi-depth"].as<size_t>();

		ofstream out(string(oprefix).c_str());
		// Output a header line
		out << "Hash\tTag\t"
			"Voro Depth\tConvolution Radius\t"
			"Ice Aeff (um)\tMax Diameter (mm)\t"
			"Ice Volume (mm^3)\tar\tvf cs\tvf ce\tvf vint\tvf hon\t"
			"vf ctot\tvf cint\tvf cext\tintmfrac"
			<< std::endl;

		auto processShape = [&](boost::shared_ptr<rtmath::ddscat::shapefile::shapefile> s)
		{
			try {
				mylog("Processing shape " << s->hash().lower);
				cerr << "  Shape " << s->hash().lower << endl;
				s->loadHashLocal(); // Load the shape fully, if it was imported from a database
				auto stats = rtmath::ddscat::stats::shapeFileStats::genStats(s);
				if (dSpacing && !s->standardD) s->standardD = (float)dSpacing;

				flakedata r;
				r.hash = s->hash().string();
				r.aeff_um = stats->aeff_dipoles_const * s->standardD;
				r.vol_mm3 = std::pow(r.aeff_um/1000.,3.);
				r.md_mm = stats->max_distance * s->standardD / 1000.;
				r.ar = 1 / stats->calcStatsRot(0, 0, 0)->get<1>().at(rtmath::ddscat::stats::rotColDefs::AS_ABS)(0, 2);
				boost::shared_ptr<rtmath::Voronoi::VoronoiDiagram> vd;
				vd = s->generateVoronoi(
					std::string("standard"),
					rtmath::Voronoi::VoronoiDiagram::generateStandard);
				auto surfdepth = vd->calcSurfaceDepth();
				vd->calcCandidateConvexHullPoints();
				auto cellmap = vd->getCellMap();

				size_t numLatticeTotal = 0, numLatticeFilled = 0;
				vd->calcFv(int_voro_depth, numLatticeTotal, numLatticeFilled);
				r.intmfrac = (double) (numLatticeFilled) / (double) (s->numPoints);

				r.vf_cs = stats->Scircum_sphere.f;
				r.vf_ce = stats->Sellipsoid_max.f;
				r.vf_vint = (double)numLatticeFilled / (double)numLatticeTotal;
				r.vf_hon = r.vf_vint * ( 1. - (0.33333*(r.md_mm / 10.)));

				double crad = vm["convolution-radius"].as<double>();
				double cthres = vm["convolution-threshold"].as<double>();

				// vd is the already-calculated Voronoi diagram. I want to perform
				// the convolution and report the overall volume fraction. I then
				// want to decompose the convolution into two spatial regions:
				// interior and exterior, depending on the Voronoi depth at each
				// point. Then, I will calculate the statistics of the convolved
				// regions (means, standard deviations and histograms).
				auto ptsearch = ::rtmath::ddscat::points::points::generate(
					s->latticePts);
				using namespace std::placeholders;
				decimationFunction df;
				df = std::bind(
					::rtmath::ddscat::points::points::convolutionNeighborsRadius,
					std::placeholders::_1,std::placeholders::_2,crad,ptsearch);
				auto cnv = ::rtmath::ddscat::points::convolute_A(
					s, ((size_t) crad) + 0.01 );
				// Iterate over all points, and average the dielectric values.
				auto volProvider = rtmath::ddscat::points::sphereVol::generate(crad + 0.01);
				double volExact = (double) volProvider->pointsInSphere();
				double volFrm = volProvider->volSphere();

				// For the next step, I need a matrix of all possible point coordinates,
				// the voronoi depth at each coordinate, and the convoluted volume fraction.
				Eigen::Array3i mins = s->mins.cast<int>() - crad,
					maxs = s->maxs.cast<int>() + crad;
				Eigen::Array3i span = maxs - mins + 1;
				int imax = span.prod();
				// Define some mapping functions to get a consistent index
				auto getIndex = [&](Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> i) -> int
				{
					// Iterate first over z, then y, then x
					int res = 0;
					i(0) = i(0) - mins(0);
					i(1) = i(1) - mins(1);
					i(2) = i(2) - mins(2);
					res = span(0) * span(1) * i(2);
					res += span(0) * i(1);
					res += i(0);
					return res;
				};

				boost::shared_ptr<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> >
					mat(new Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>);
				// Coordinate, then num neighbors, then vf, then voro depth
				mat->resize(imax, 6); //cellmap->rows(), 6);
				mat->setConstant(-1);
				// Iterate first over the voronoi diagrams
				for (int i=0; i < cellmap->rows(); ++i)
				{
					Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> pt;
					pt.resize(1,3);
					pt = cellmap->cast<float>().block<1,3>(i,0);
					int matId = getIndex(pt); // mat table row number
					int voroId = (*cellmap)(i,3); // Depth table row number
					// Fill in the point
					mat->block<1,3>(matId,0) = pt;
					// Fill in the data
					if (voroId >= 0)
						(*mat)(matId, 5) = (*surfdepth)(voroId, 3);
					else (*mat)(matId, 5) = -1.f;
				}
				// Now iterate over the convolution results
				for (int i=0; i < cnv->latticePts.rows(); ++i) {
					Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> pt;
					pt.resize(1,3);
					pt(0,0) = cnv->latticePts(i,0); pt(0,1) = cnv->latticePts(i,1);
					pt(0,2) = cnv->latticePts(i,2);
					int matId = getIndex(pt); // mat table row number
					// Fill in the data
					(*mat)(matId, 3) = cnv->latticePtsRi(i,0);
					(*mat)(matId, 4) = (*mat)(matId, 3) / volExact;
				}
				int ntot = 0, nint = 0, next = 0, nign = 0;
				double sumtot = 0, sumint = 0, sumext = 0, sumign = 0;
				// Finally, iterate over mat
				for (int i=0; i < mat->rows(); ++i) {
					if ((*mat)(i, 4) < cthres) {
						// Ignore point
						nign++;
						sumign += (*mat)(i, 4);
					} else {
						if ((*mat)(i,5) < int_voro_depth) {
							// Surface point
							next++;
							sumext += (*mat)(i, 4);
							sumtot += (*mat)(i, 4);
						} else {
							// Interior point
							nint++;
							sumint += (*mat)(i, 4);
							sumtot += (*mat)(i, 4);
						}
					}
				}

				r.vf_ctot = sumtot / (double) (nint + next);
				r.vf_cint = sumint / (double) nint;
				r.vf_cext = sumext / (double) next;
				cout << "N " << nint + next << " int " << nint << " ext " << next << endl;
				out << r.hash << "\t" << tag << "\t"
					<< int_voro_depth << "\t" << crad << "\t"
					<< r.aeff_um << "\t"
					<< r.md_mm << "\t" << r.vol_mm3 << "\t" << r.ar << "\t"
					<< r.vf_cs << "\t" << r.vf_ce << "\t"
					<< r.vf_vint << "\t" << r.vf_hon << "\t"
					<< r.vf_ctot << "\t" << r.vf_cint << "\t"
					<< r.vf_cext << "\t" << r.intmfrac << endl;

			} catch (Ryan_Debug::error::xMissingHash &err) {
				cerr << "  Cannot find hash for " << s->hash().lower << endl;
				cerr << err.what() << endl;
			}
		};

		for (const auto &s : *(res.first))
			processShape(rtmath::ddscat::shapefile::shapefile::generate(s));

	}
	catch (std::exception &e) {
		cerr << "Exception: " << e.what() << endl;
		exit(2);
	}
	catch (...) {
		cerr << "Caught unidentified error... Terminating." << endl;
		exit(1);
	}

	return 0;
}



