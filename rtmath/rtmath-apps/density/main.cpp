/// Density program tablulates various particle size / density relations.
/// It accounts for temperature and different length scale units.
#include <iostream>
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
#include "../../rtmath/rtmath/density.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/registry.h"
#include "../../rtmath/rtmath/error/debug.h"

int main(int argc, char *argv[])
{
	using namespace std;
	try {
		cerr << "rtmath-density" << endl;
		// Do processing of argv
		namespace po = boost::program_options;
		po::options_description desc("Allowed options"), cmdline("Command-line options"),
			runmatch("Run-matching options"), basic("Basic ellipsoid options"),
			refract("Density options"), scale("Scaling options"),
			config("Config options"), hidden("Hidden options"), oall("all options");

		rtmath::debug::add_options(cmdline, config, hidden);
		rtmath::ddscat::stats::shapeFileStats::add_options(cmdline, config, hidden);
		//rtmath::ddscat::ddOutput::add_options(cmdline, config, hidden);
		//rtmath::refract::add_options(cmdline, config, hidden);

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
			;
		refract.add_options()
			("temps,T", po::value<std::string>()->default_value("263"), "Specify temperatures in K")
			("volume-fractions,v", po::value<std::string>()->default_value("1"), "Set the ice volume fractions. Used when not doing shapefiles.")
			("vf-scaling", po::value<string>()->default_value("Circumscribing_Sphere"),
			"Select the method used in determining the volume fraction."
			" 1) Voronoi_Internal uses the internal volume fraction,"
			" 2) Circumscribing_Sphere, 3) Convex, 4) Ellipsoid_Max uses the max circumscribing ellipsoid,"
			" 5) RMS_Sphere for a root mean square sphere (per Petty and Huang 2010),"
			" 6) Gyration_Sphere uses the Westbrok 2006 radius of gyration,"
			" 7) Solid_Sphere is an optically hard sphere,"
			" 8) Ellipsoid_Max_Holly is another implementation of the max circumscribing ellipsoid,"
			" 9) Brandes2007 impelements the Brandes et al. (2007) eqn. 7 relation,"
			" 10) MagonoNakamura1965,"
			" 11) Holroyd1971,"
			" 12) Muramoto1995,"
			" 13) FabrySzyrmer1999,"
			" 14) Heymsfield2004,"
			" 15) BrownFrancis1995Hogan2012."
			)
			("voronoi-depth", po::value<size_t>()->default_value(2), "Sets the internal voronoi depth for scaling")
			;
		basic.add_options()
			("aspect-ratios,s", po::value<std::string>()->default_value("1"), "Specify aspect ratio for ellipsoids.")
			("aeffs,a", po::value<std::string>(), "Specify the effective radii in um")
			("radii,r", po::value<std::string>(), "Specify the actual mean sphere radii in um")
			;
		scale.add_options()
			;

		cmdline.add_options()
			("help,h", "produce help message")
			("output-prefix,o", po::value<string>()->default_value("output"), "Set output file (required)")
			;

		desc.add(cmdline).add(config).add(runmatch).add(refract).add(basic).add(scale);
		oall.add(cmdline).add(runmatch).add(refract).add(config).add(hidden).add(basic).add(scale);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);
		rtmath::ddscat::stats::shapeFileStats::process_static_options(vm);

		// Begin checking parameters
		auto doHelp = [&](const std::string& s) {
			cout << s << endl;
			cout << desc << endl;
			exit(3);
		};

		if (vm.count("help") || vm.size() == 0) doHelp("");

		string oprefix;
		if (vm.count("output-prefix"))
			oprefix = vm["output-prefix"].as<string>();

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
		bool matchParentFlakes;

		if (vm.count("match-hash")) matchHashes = vm["match-hash"].as<vector<string> >();
		if (vm.count("match-flake-type")) matchFlakeTypes = vm["match-flake-type"].as<vector<string> >();
		if (vm.count("match-parent-flake-hash")) matchParentHashes = vm["match-parent-flake-hash"].as<vector<string> >();
		if (vm.count("match-dipole-spacing")) iDipoleSpacing.append(vm["match-dipole-spacing"].as<vector<string>>());
		if (vm.count("match-dipole-numbers")) iDipoleNumbers.append(vm["match-dipole-numbers"].as<vector<string>>());

		if (vm.count("match-parent-flake")) matchParentFlakes = true;

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



		struct run
		{
			double temp, aeff, ar;
			double fv;
			std::string fvMeth;
			std::string refHash;
		};
		vector<run> runs;
		runs.reserve(50000);

		// First, set any overrides for the frequency, temperature and refractive index
		set<double> temps;
		if (vm.count("temps"))
			Ryan_Debug::splitSet::splitSet(vm["temps"].as<string>(), temps);

		enum class VFRAC_TYPE
		{
			CIRCUM_SPHERE,
			VORONOI,
			CONVEX,
			ELLIPSOID_MAX,
			INTERNAL_VORONOI,
			RMS_SPHERE,
			GYRATION_SPHERE,
			SOLID_SPHERE,
			ELLIPSOID_MAX_HOLLY,
			OTHER
		};
		VFRAC_TYPE vf = VFRAC_TYPE::CIRCUM_SPHERE;
		std::string vfother;
		string vfScaling = vm["vf-scaling"].as<string>();

		if (vfScaling == "Circumscribing_Sphere") vf = VFRAC_TYPE::CIRCUM_SPHERE;
		else if (vfScaling == "Voronoi_Full") vf = VFRAC_TYPE::VORONOI;
		else if (vfScaling == "Voronoi_Internal") vf = VFRAC_TYPE::INTERNAL_VORONOI;
		else if (vfScaling == "Convex") vf = VFRAC_TYPE::CONVEX;
		else if (vfScaling == "Ellipsoid_Max") vf = VFRAC_TYPE::ELLIPSOID_MAX;
		else if (vfScaling == "Ellipsoid_Max_Holly") vf = VFRAC_TYPE::ELLIPSOID_MAX_HOLLY;
		else if (vfScaling == "RMS_Sphere") vf = VFRAC_TYPE::RMS_SPHERE;
		else if (vfScaling == "Gyration_Sphere") vf = VFRAC_TYPE::GYRATION_SPHERE;
		else if (vfScaling == "Solid_Sphere") vf = VFRAC_TYPE::SOLID_SPHERE;
		else { vf = VFRAC_TYPE::OTHER; vfother = vfScaling; }
		size_t int_voro_depth = vm["voronoi-depth"].as<size_t>();

		auto process_commandline = [&]()
		{
			// If any of these are set, ensure that the temperature and frequency are also set

			// Freqs and temps are in main's scope
			set<double> aeffs, radii, aspects, vfracs; // , alphas, betas;
			if (vm.count("aeffs"))
				Ryan_Debug::splitSet::splitSet(vm["aeffs"].as<string>(), aeffs);
			if (vm.count("radii"))
				Ryan_Debug::splitSet::splitSet(vm["radii"].as<string>(), radii);
			if (vm.count("aspect-ratios"))
				Ryan_Debug::splitSet::splitSet(vm["aspect-ratios"].as<string>(), aspects);
			if (vm.count("volume-fractions"))
				Ryan_Debug::splitSet::splitSet(vm["volume-fractions"].as<string>(), vfracs);

			if (!temps.size() && !overrideM) doHelp("Need to specify temperatures.");
			if ((aeffs.size() || radii.size()) && aspects.size() && vfracs.size())
			{
				if (temps.size() || overrideM)
				{
						for (const auto &aspect : aspects)
							for (const auto &vfrac : vfracs)
							{
								auto doAeff = [&](double aeff, bool rescale)
								{
									if (rescale) {
										double V_rad = std::pow(aeff,3.);
										double V_sca = V_rad * vfrac;
										aeff = std::pow(V_sca, 1./3.);
									}
									if (overrideM)
									{
										run r;
										r.aeff = aeff;
										r.ar = aspect;
										r.fv = vfrac;
										r.temp = -1;
										runs.push_back(std::move(r));
									}
									else {
										for (const auto &temp : temps)
										{
											run r;
											r.aeff = aeff;
											r.ar = aspect;
											r.fv = vfrac;
											r.temp = temp;
											runs.push_back(std::move(r));
										}
									}
								};
								for (const auto &aeff : aeffs)
									doAeff(aeff, false);
								for (const auto &rad : radii)
								{
									// Rescale radius to effective radius
									// using r.fv, rad.
									//double V_rad = std::pow(rad,3.);
									//double V_sca = V_rad * r.fv;
									//double aeff = std::pow(V_sca, 1./3.);
									doAeff(rad, true);
								}
					}
				}
				else doHelp("Need to specify temperatures or a refractive index");
			}
		};

		process_commandline();



		auto processShape = [&](boost::shared_ptr<rtmath::ddscat::shapefile::shapefile> s)
		{
			try {
				cerr << "  Shape " << s->hash().lower << endl;
				s->loadHashLocal(); // Load the shape fully, if it was imported from a database
				auto stats = rtmath::ddscat::stats::shapeFileStats::genStats(s);
				if (dSpacing && !s->standardD) s->standardD = (float)dSpacing;

					for (const auto &temp : temps) { // Can also be -1 when overriding temp
					run r;

					rtmath::ddscat::stats::shapeFileStatsBase::volumetric *v = nullptr;
					if (vf == VFRAC_TYPE::CIRCUM_SPHERE) {
						v = &(stats->Scircum_sphere); r.fvMeth = "Circumscribing Sphere";
					} else if (vf == VFRAC_TYPE::RMS_SPHERE) {
						v = &(stats->Srms_sphere); r.fvMeth = "RMS Sphere";
					} else if (vf == VFRAC_TYPE::GYRATION_SPHERE) {
						v = &(stats->Sgyration); r.fvMeth = "Radius of gyration Sphere";
					} else if (vf == VFRAC_TYPE::VORONOI) {
						v = &(stats->SVoronoi_hull); r.fvMeth = "Voronoi hull";
					} else if (vf == VFRAC_TYPE::CONVEX) {
						v = &(stats->Sconvex_hull); r.fvMeth = "Convex hull";
					} else if (vf == VFRAC_TYPE::ELLIPSOID_MAX) {
						v = &(stats->Sellipsoid_max);
						r.fvMeth = "Max ellipsoid";
					} else if (vf == VFRAC_TYPE::ELLIPSOID_MAX_HOLLY) {
						v = &(stats->Sellipsoid_max_Holly);
						r.fvMeth = "Max ellipsoid Holly";
					} else if (vf == VFRAC_TYPE::SOLID_SPHERE) {
						v = &(stats->Ssolid);
						r.fvMeth = "Solid sphere";
					} else if ((vf == VFRAC_TYPE::INTERNAL_VORONOI) && (int_voro_depth == 2)) {
						v = &(stats->SVoronoi_internal_2);
						r.fvMeth = "Internal Voronoi Depth 2";
					} else if ((vf == VFRAC_TYPE::OTHER)) {
						r.fvMeth = vfother;
					}
					if (v) {
						r.aeff = v->aeff_V * s->standardD;
						r.fv = v->f;
					} else if (vf == VFRAC_TYPE::INTERNAL_VORONOI) {
						boost::shared_ptr<rtmath::Voronoi::VoronoiDiagram> vd;
						vd = s->generateVoronoi(
							std::string("standard"), rtmath::Voronoi::VoronoiDiagram::generateStandard);
						vd->calcSurfaceDepth();
						vd->calcCandidateConvexHullPoints();

						size_t numLatticeTotal = 0, numLatticeFilled = 0;
						vd->calcFv(int_voro_depth, numLatticeTotal, numLatticeFilled);

						r.aeff = stats->aeff_dipoles_const * s->standardD;
						r.fvMeth = "Internal Voronoi Depth ";
						r.fvMeth.append(boost::lexical_cast<std::string>(int_voro_depth));
						r.fv = (double)numLatticeFilled / (double)numLatticeTotal;
					} else RDthrow(Ryan_Debug::error::xBadInput())
						<< Ryan_Debug::error::otherErrorText("Unhandled volume fraction method");

					/// Gives aspect ratio of matching ellipsoid
					/// \todo Implement this function
					auto arEllipsoid = [](double sa, double v) -> double
					{
						RDthrow(Ryan_Debug::error::xUnimplementedFunction());
						return -1;
					};


					if (armeth == "Max_Ellipsoids")
						// 0,2 to match Holly convention!!!!!!! 0,1 always is near sphere.
						r.ar = stats->calcStatsRot(0, 0, 0)->get<1>().at(rtmath::ddscat::stats::rotColDefs::AS_ABS)(0, 2);
					else if (armeth == "Spheres")
						r.ar = 1;
					else doHelp("ar-method needs a correct value.");
					r.freq = freq;
					if (overrideM) r.m = ovM;
					else rtmath::refract::mIce(freq, temp, r.m);
					r.temp = temp;
					r.refHash = s->hash().string();
					r.lambda = rtmath::units::conv_spec("GHz", "um").convert(freq);

					runs.push_back(std::move(r));
					}
				} catch (Ryan_Debug::error::xMissingHash &err) {
					cerr << "  Cannot find hash for " << s->hash().lower << endl;
					cerr << err.what() << endl;
				}
		};

		for (const auto &s : *(res.first))
			processShape(rtmath::ddscat::shapefile::shapefile::generate(s));




		ofstream out(string(oprefix).c_str());
		// Output a header line
		out << "Aeff (um)\tMax_Diameter (um)\tVolume Fraction\tDensity (g/cm^3)\tTemperature (K)\t"
			"Hash\tAspect Ratio\tAR Method\tVolume Fraction Method\t"
			"Method" << std::endl;

		std::cerr << "Doing " << runs.size() << " calculations." << std::endl;
		cout << "Meth\tg\tQabs\tQbk\tQext\tQsca" << std::endl;
		size_t i=0;
		// Iterate over all possible runs
		for (const auto &r : runs)
		{
			ostringstream ofiless;
			// the prefix is expected to be the discriminant for the rotational data
			ofiless << oprefix << "-t-" << r.temp << "-f-" << r.freq
				<< "-aeff-" << r.aeff
				<< "-vfrac-" << r.fv << "-aspect-" << r.ar;
			string ofile = ofiless.str();
			cout << ofile << " --- " << i << " " << r.refHash << endl;
			++i;

				out << rr.first << "\t" << r.aeff << "\t" << r.freq << "\t" << r.fv << "\t" << r.temp << "\t"
					<< r.refHash << "\t" << armeth << "\t" << refractScaling << "\t" << r.fvMeth << "\t"
					<< r.ar << "\t" << r.lambda << "\t" << r.m.real() << "\t" << r.m.imag() << "\t"
					<< sizep << "\t" << i.aeff_rescale << "\t"
					<< s.theta << "\t" << s.beta << "\t" << s.phi << "\t"
					<< rr.second.g_iso << "\t"
					<< rr.second.Qabs_iso << "\t"
					<< rr.second.Qbk_iso << "\t"
					<< rr.second.Qext_iso << "\t"
					<< rr.second.Qsca_iso
					<< std::endl;

				cout << rr.first << "\t"
					<< rr.second.g_iso << "\t"
					<< rr.second.Qabs_iso << "\t"
					<< rr.second.Qbk_iso << "\t"
					<< rr.second.Qext_iso << "\t"
					<< rr.second.Qsca_iso
					<< std::endl;
		}

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

