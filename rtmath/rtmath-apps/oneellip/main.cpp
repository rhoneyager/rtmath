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
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/density.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/phaseFunc.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/registry.h"
#include "../../rtmath/rtmath/error/debug.h"

int main(int argc, char *argv[])
{
	using namespace std;
	try {
		cerr << "rtmath-oneellip" << endl;
		// Do processing of argv
		namespace po = boost::program_options;
		po::options_description desc("Allowed options"), cmdline("Command-line options"),
			runmatch("Run-matching options"), basic("Basic ellipsoid options"),
			refract("Refractive index options"), scale("Scaling options"),
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
			" 15) BrownFrancis1995Hogan2012.")
			("voronoi-depth", po::value<size_t>()->default_value(2), "Sets the internal voronoi depth for scaling")

			("ar-method", po::value<string>()->default_value("Max_Ellipsoids"),
			"Max_Ellipsoids: Force aspect ratios to be ellipsoids, following the max AR calculated in stats code. "
			"Spheres: Force aspect ratios to be spheres, instead of stats-determined spheroids. "
			"(TODO SA_V_Ellipsoids: Select AR of ellipsoid with matching surface area to volume ratio).")
			;
		refract.add_options()
			("refract-method", po::value<string>()->default_value("Maxwell_Garnett_Ellipsoids"), "Method used to calculate the resulting dielectric "
			"(Sihvola, Debye, Maxwell_Garnett_Spheres, Maxwell_Garnett_Ellipsoids, Bruggeman). "
			"Only matters if volume fractions are given. Then, default is Maxwell_Garnett_Ellipsoids.")
			("temps,T", po::value<std::string>()->default_value("263"), "Specify temperatures in K")
			("mr", po::value<double>(), "Override real refractive index value")
			("mi", po::value<double>(), "Override imaginary refractive index value")
			("volume-fractions,v", po::value<std::string>(), "Set the ice volume fractions. "
			 "Used when: not using shapefiles AND vf-scaling is not one of the paper-based methods.")
			("nu,n", po::value<double>()->default_value(0.85), "Value of nu for Sihvola refractive index scaling")
			("solution-method", po::value<std::string>(), "Force only a specific algorithm to be used, such as "
			 "Rayleigh or bhmie. No other pf generator will be used.")
			;
		basic.add_options()
			("aspect-ratios,s", po::value<std::string>()->default_value("1"), "Specify aspect ratio for ellipsoids.")
			("sizes,a", po::value<std::string>(), "Specify the particle sizes in um")
			("size-type", po::value<std::string>()->default_value("Effective_Radius_Ice"), "How is the size described? "
			 "Effective_Radius_Ice, Effective_Radius_Full, Max_Diameter_Full, etc. are valid options.")
			("freqs,f", po::value<std::string>(), "Specify frequencies in GHz. Needed for dielectrics.")
			;
		scale.add_options()
			("scale-aeff", po::value<bool>()->default_value(true), "Scale effective radius based on volume fraction")
			("scale-m", po::value<bool>()->default_value(true), "Scale refractive index based on volume fraction")
			//("scale-v", "Scale for an equivalent volume ellipsoid")
			//("scale-sa", "Scale for an equivalent surface area ellipsoid")
			;

		cmdline.add_options()
			("help,h", "produce help message")
			("output-prefix,o", po::value<string>()->default_value("output"), "Set output file (required)")
			//("alphas", po::value<string>()->default_value("0"), "Set first rotation for backscatter calculation. UNDER DEVELOPMENT.")
			//("betas", po::value<string>()->default_value("0"), "Set second rotation for backscatter calculation. UNDER DEVELOPMENT.")
			//("random-rotations,r", po::value<size_t>(),
			//"Replaces standard alpha and beta angles with random points.")
			;

		desc.add(cmdline).add(config).add(runmatch).add(refract).add(basic).add(scale);
		oall.add(cmdline).add(runmatch).add(refract).add(config).add(hidden).add(basic).add(scale);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);
		rtmath::ddscat::stats::shapeFileStats::process_static_options(vm);
		//rtmath::ddscat::ddOutput::process_static_options(vm);
		//rtmath::refract::process_static_options(vm);

		// Begin checking parameters
		auto doHelp = [&](const std::string& s)
		{
			cout << s << endl;
			cout << desc << endl;
			exit(3);
		};

		if (vm.count("help") || vm.size() == 0) doHelp("");

		string smeth;
		if (vm.count("solution-method"))
			smeth = vm["solution-method"].as<string>();
		string oprefix;
		if (vm.count("output-prefix"))
			oprefix = vm["output-prefix"].as<string>();

		double dSpacing = 0;
		if (vm.count("dipole-spacing"))
			dSpacing = vm["dipole-spacing"].as<double>();

		vector<string> inputs;
		if (vm.count("input-shape"))
		{
			inputs = vm["input-shape"].as< vector<string> >();
			cerr << "Input files are:" << endl;
			for (auto it = inputs.begin(); it != inputs.end(); it++)
				cerr << "\t" << *it << "\n";
		};
		std::shared_ptr<Ryan_Debug::registry::DBhandler> dHandler;
		/*
		("from-db", "Perform search on database and select files matching criteria.")
		("match-hash", po::value<vector<string> >(), "Match lower hashes")
		("match-flake-type", po::value<vector<string> >(), "Match flake types")
		("match-dipole-spacing", po::value<vector<float> >(), "Match typical dipole spacings")
		("match-parent-flake-hash", po::value<vector<string> >(), "Match flakes having a given parent hash")
		("match-parent-flake", "Select the parent flakes")
		*/
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



		struct run
		{
			double temp, freq, aeff, lambda, ar;
			double fv;
			std::string fvMeth;
			std::complex<double> m;
			std::string refHash;
		};
		vector<run> runs;
		runs.reserve(50000);

		// First, set any overrides for the frequency, temperature and refractive index
		set<double> temps, freqs;
		if (vm.count("temps"))
			Ryan_Debug::splitSet::splitSet(vm["temps"].as<string>(), temps);
		if (vm.count("freqs"))
			Ryan_Debug::splitSet::splitSet(vm["freqs"].as<string>(), freqs);
		bool overrideM = false;
		complex<double> ovM;
		double nu = vm["nu"].as<double>();
		if (vm.count("mr") || vm.count("mi"))
		{
			overrideM = true;
			if (!vm.count("mr") || !vm.count("mi"))
				doHelp("When overriding refractive index, need to specify "
				"both real and imaginary components.");
			double r = vm["mr"].as<double>();
			double i = vm["mi"].as<double>();
			ovM = complex<double>(r, i);
			nu = -1.0;
		}
		auto sihvolaBinder = [&](std::complex<double> Ma, std::complex<double> Mb, double fa, std::complex<double> &Mres)
		{
			rtmath::refract::sihvola(Ma, Mb, fa, nu, Mres);
		};
		auto fixedBinder = [&](std::complex<double>, std::complex<double>, double, std::complex<double> &Mres)
		{
			Mres = ovM;
		};


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
		else vf = VFRAC_TYPE::OTHER;
		size_t int_voro_depth = vm["voronoi-depth"].as<size_t>();

		bool rescaleM = vm["scale-m"].as<bool>();

		/*
		("refract-method", po::value<string>()->default_value("Maxwell_Garnett_Ellipsoids"), "Method used to calculate the resulting dielectric "
		"(Sihvola, Debye, Maxwell_Garnett_Spheres, Maxwell_Garnett_Ellipsoids). "
		"Only matters if volume fractions are given, default is Maxwell_Garnett_Ellipsoids.")
		*/
		string refractScaling = vm["refract-method"].as<string>();
		std::function<void(std::complex<double>, std::complex<double>, double, std::complex<double> &)> rmeth;
		if (refractScaling == "Sihvola") { 
			rmeth = sihvolaBinder;
			refractScaling.append(" nu ");
			refractScaling.append(boost::lexical_cast<std::string>(nu));
		} else if (refractScaling == "Maxwell_Garnett_Ellipsoids") rmeth = rtmath::refract::maxwellGarnettEllipsoids;
		else if (refractScaling == "Maxwell_Garnett_Spheres") rmeth = rtmath::refract::maxwellGarnettSpheres;
		else if (refractScaling == "Debye") rmeth = rtmath::refract::debyeDry;
		else if (refractScaling == "Bruggeman") rmeth = rtmath::refract::bruggeman;
		else if (!rescaleM) rmeth = fixedBinder;
		else doHelp("Need to specify a proper refractive  scaling.");

		string armeth = vm["ar-method"].as<string>(); // Used much further below


		auto process_commandline = [&]()
		{
			// If any of these are set, ensure that the temperature and frequency are also set

			// Freqs and temps are in main's scope
			string sizetype = vm["size-type"].as<string>();
			set<double> aeffs, aspects, vfracs; // , alphas, betas;
			if (vm.count("sizes"))
				Ryan_Debug::splitSet::splitSet(vm["sizes"].as<string>(), aeffs);
			if (vm.count("aspect-ratios"))
				Ryan_Debug::splitSet::splitSet(vm["aspect-ratios"].as<string>(), aspects);
			if (vm.count("volume-fractions"))
				Ryan_Debug::splitSet::splitSet(vm["volume-fractions"].as<string>(), vfracs);

			// To make life easier, I no longer have to explicitly specify volume
			// fractions. If one of the supported scaling methods is selected (one
			// that is not exact shape-dependent), then the volume fractions can
			// be automatically determined.
			if (!freqs.size()) doHelp("Need to specify frequencies.");
			if (!temps.size() && !overrideM) doHelp("Need to specify temperatures.");

			if (freqs.size() && (aeffs.size() ) && aspects.size())
			{
				if (temps.size() || overrideM)
				{
					for (const auto &freq : freqs)
						for (const auto &aspect : aspects) {
							auto doAeff = [&](double aeff, double vfrac, double temp)
							{
								run r;
								r.aeff = aeff;
								r.ar = aspect;
								r.freq = freq;
								r.fv = vfrac;
								r.lambda = rtmath::units::conv_spec("GHz", "um").convert(freq);
								r.m = ovM;
								r.temp = temp;
								if (vf == VFRAC_TYPE::OTHER) {
									r.fvMeth = vfScaling;
								}
								if (overrideM)
									r.m = ovM;
								else
									rtmath::refract::mIce(freq, temp, r.m);
								runs.push_back(std::move(r));
							};
							std::vector<std::tuple<double, double, double> > aeff_vf_rad;
							// If aspect ratios are specified, then a custom run is indicated.
							if (!vfracs.size() && vf != VFRAC_TYPE::OTHER)
								doHelp("Need to specify how volume fractions are determined.");
							// Density is temperature-dependent.
							if (!vfracs.size() && vf == VFRAC_TYPE::OTHER)
							{
								if (!temps.size()) doHelp("When using a predetermined density "
									"formula, temperature should be specified.");
								for (const auto &temp : temps) {
								// Populate the volume fractions being calculated
									for (const auto &aeff : aeffs) {
										using namespace rtmath::density;
										double dIce = ice1h( _temperature = temp, _temp_units = "K" );
										double dEff = 0;
										dEff = effDen(
											_provider = vfScaling,
											_in_length_value = aeff,
											_in_length_type = sizetype, //"Effective_Radius_Ice",
											_in_length_units = "um",
											_ar = aspect,
											_temperature = temp,
											_temp_units = "K"
											);
										bool isIceAeff = false;
										double vf = dEff / dIce;
										double V = pow(aeff,3.);
										if (sizetype.find("Ice") != std::string::npos) isIceAeff = true;
										double inAeff = 0;
										double rad = 0;
										if (isIceAeff) {
											inAeff = aeff;
											V /= vf;
											rad = pow(V,1./3.);
										} else {
											rad = aeff;
											V *= vf;
											inAeff = pow(V,1./3.);
										}
										std::cerr << "aeff " << inAeff << " rad " << rad << " dIce " << dIce << " dEff " << dEff << " vf " << vf << std::endl;
										aeff_vf_rad.push_back(std::tuple<double, double, double>(inAeff, vf, temp));
									}
								}
							} else {
								if (!temps.size() && overrideM) temps.insert(-1);
								for (const auto &temp : temps)
									for (const auto &vf : vfracs) {
										for (const auto &aeff : aeffs)
											aeff_vf_rad.push_back(std::tuple<double, double, double>
												(aeff, vf, temp));
								}
							}
							for (const auto &t : aeff_vf_rad)
								doAeff(std::get<0>(t), std::get<1>(t), std::get<2>(t));
						}
				}
				else doHelp("Need to specify temperatures or a refractive index");
			}
			/*
			else if (aeffs.size() || radii.size() || aspects.size() || vfracs.size())
			{
				std::ostringstream o;
				o << "Need to specify: ";
				if (!aeffs.size() && !radii.size()) o << "aeffs or radii, ";
				if (!aspects.size()) o << "aspect ratios, ";
				if (!vfracs.size()) o << "volume fractions, ";
				if (!freqs.size()) o << "frequencies, ";
				if (!overrideM && !temps.size()) o << "temperatures or dielectric overloads";
				doHelp(o.str());
			}*/

		};

		process_commandline();



		auto processShape = [&](boost::shared_ptr<rtmath::ddscat::shapefile::shapefile> s)
		{
			try {
				cerr << "  Shape " << s->hash().lower << endl;
				s->loadHashLocal(); // Load the shape fully, if it was imported from a database
				auto stats = rtmath::ddscat::stats::shapeFileStats::genStats(s);
				if (dSpacing && !s->standardD) s->standardD = (float)dSpacing;

				if (overrideM) { temps.clear(); temps.insert(-1); } // Use a dummy temperatre value so that the loop works.
				for (const auto &freq : freqs)
					for (const auto &temp : temps) { // Can also be -1 when overriding temp
					run r;

					if (armeth == "Max_Ellipsoids")
						// 0,2 to match Holly convention!!!!!!! 0,1 always is near sphere.
						r.ar = stats->calcStatsRot(0, 0, 0)->get<1>().at(rtmath::ddscat::stats::rotColDefs::AS_ABS)(0, 2);
					else if (armeth == "Spheres")
						r.ar = 1;
					else doHelp("ar-method needs a correct value.");

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
					} else if (vf == VFRAC_TYPE::OTHER) {
						r.fvMeth = vfScaling;
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
					} else if (vf == VFRAC_TYPE::OTHER ) {
						// TODO: Determine r.fv based on the density scaling relation
						// Effective radius and aspect ratio are known, so the other quantities like 
						// effectvie mac dimension are easily determined. Assume that ice is used, 
						// like in the refractive index portion of the code.
						using namespace rtmath::density;
						double dIce = ice1h( _temperature = temp, _temp_units = "K" );
						double dEff = 0;
						dEff = effDen(
							_provider = r.fvMeth,
							_in_length_value = r.aeff,
							_in_length_type = "Effective_Radius_Ice",
							_in_length_units = "um",
							_ar = r.ar,
							_temperature = temp,
							_temp_units = "K"
							);
						r.fv = dEff / dIce;
					}

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
		out << "Cross-Section Method\tAeff (um)\tMax Diameter (mm)\t"
			"Ice Volume (mm^3)\tFrequency (GHz)\t"
			"Volume Fraction\tEffective Density (g/cm^3)\t"
			"Temperature (K)\tHash\t"
			"AR Method\tRefractive Index Method\tVolume Fraction Method\t"
			"Aspect Ratio\tLambda (um)\tM_re\tM_im\t"
			"Size Parameter\tRescale aeff\t"
			"Theta\tBeta\tPhi\tg\tQabs\tQbk\tQext\tQsca" << std::endl;

		std::cerr << "Doing " << runs.size() << " runs." << std::endl;
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

			const double sizep = 2. * boost::math::constants::pi<double>() * r.aeff / r.lambda;

			using namespace rtmath::phaseFuncs;
			pf_class_registry::orientation_type o = pf_class_registry::orientation_type::ISOTROPIC;
			pf_class_registry::inputParamsPartial i;
			i.aeff = r.aeff;
			i.aeff_rescale = vm["scale-aeff"].as<bool>();
			i.aeff_version = pf_class_registry::inputParamsPartial::aeff_version_type::EQUIV_V_SPHERE;
			i.eps = r.ar;
			i.m = r.m;
			i.ref = r.refHash;
			//std::cerr << "i.m = r.m = " << i.m << std::endl;
			i.rmeth = rmeth; // Yeah, only one refractive index method per program invocation is supported.
			i.shape = pf_class_registry::inputParamsPartial::shape_type::SPHEROID;
			i.vFrac = r.fv;

			if (r.fv <= 0 || r.fv > 1.) {
				std::cout << "\tCowardly refusing to calculate scattering for an invalid particle. See "
					"the log for details (channel=density). Skipping to next one." << std::endl;
				continue;
			}


			pf_provider p(o, i);

			pf_provider::resCtype res;
			pf_class_registry::setup s;
			s.beta = 0; s.theta = 0; s.phi = 0;
			s.sPhi = 0; s.sPhi0 = 0; s.sTheta = 0; s.sTheta0 = 0;
			s.wavelength = r.lambda;

			// smeth is parameter solution-method, which can force a single method to be used
			p.getCrossSections(s, res, smeth);

			const double pi = boost::math::constants::pi<double>();
			double maxDiam = 0, Vi = 0, V = 0, effDen = 0;
			{
				using namespace rtmath::density;
				double aeffmm = convertLength( _ar = r.ar,
					_in_length_units = "um",
					_out_length_units = "mm",
					_in_length_value = r.aeff,
					_in_length_type = "Max_Diameter_Ice",
					_out_length_type = "Max_Diameter_Ice"
					);
				V = (4. * pi / 3.) * pow(aeffmm,3.);
				Vi = V;
				V /= r.fv;
				double rad = pow(3.*V/(4.*pi),1./3.);
				maxDiam = 2. * rad;

				double dIce = ice1h( _temperature = r.temp, _temp_units = "K" );
				effDen = dIce * r.fv;
			}

			for (const auto &rr : res)
			{
				out << rr.first << "\t" << r.aeff << "\t" << maxDiam << "\t" 
					<< Vi << "\t"
					<< r.freq << "\t" << r.fv << "\t" << effDen << "\t" << r.temp << "\t"
					<< r.refHash << "\t" << armeth << "\t" << refractScaling << "\t" 
					<< r.fvMeth << "\t" //<< r.fvMeth * 0 << "\t" // get ice density here.....
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



