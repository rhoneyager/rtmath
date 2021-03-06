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
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/density/density.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/conversions/convertLength.h"
#include "../../rtmath/rtmath/phaseFunc.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
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
		cerr << "rtmath-oneellip" << endl;
		const double pi = boost::math::constants::pi<double>();
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
			" 15) BrownFrancis1995Hogan2012,"
			" 16) Constant lets you specify an exact volume fraction (see constant-vf-shape),"
			" 17) Voronoi_Offset takes the Voronoi vf and subtracts 0.1. It's a suggested correction in my paper."
			" 18) Linear specifies linear volume fractions with two known anchor points. See vf-points options."
			)
			("voronoi-depth", po::value<size_t>()->default_value(2), "Sets the internal voronoi depth for scaling")
			("voronoi-offset-factor", po::value<double>()->default_value(3), "Sets the factor used in the Voronoi offset "
			 "method")
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
			("vf-aeff-1", po::value<double>(), "")
			("vf-aeff-2", po::value<double>(), "")
			("vf-val-1", po::value<double>(), "")
			("vf-val-2", po::value<double>(), "")
//			("vf-is-den", "The -v parameter is not a volume fraction. It is a density in g/cm^3.")
//			("constant-vf-shape", po::value<double>()->default_value(0), "Use this option to "
//			 "force a constant volume fraction for use in DDA comparison runs. The regular "
//			 "volume-fractions parameter will not work in this case.")
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
			("other-param,p", po::value<std::vector<std::string> >(), "Specify optional string parameters "
			 "that will be passed to the backend functions. Give in type:key:value notation. "
			 "Types are double or string.")
			("show-qs", "Instead of actual cross sections, show Q values")
			("use-ddscat-conv", "Use ddscat conventions for backscatter")
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

		auto& lg = m_app::get();
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

		// Let's understand the other parameters.
		auto otherOpts = Ryan_Debug::registry::options::generate();
		if (vm.count("other-param")) {
			vector<string> oopts = vm["other-param"].as<vector<string> >();
			// Split based to key and value based on presence of ':'.
			for (const auto &c : oopts) {
				vector<string> ssplit;
				Ryan_Debug::splitSet::splitVector(c, ssplit, ':');
				if (ssplit.size() != 2)
					RDthrow(Ryan_Debug::error::xBadInput()) 
						<< Ryan_Debug::error::key(c)
						<< Ryan_Debug::error::otherErrorText("other-param needs type:key:value format.");
				// For now, all option types use the same format.
				otherOpts->setVal<std::string>(ssplit[0], ssplit[1]);
			}
		}

		double vf_aeff_1 = -1, vf_aeff_2 = -1, vf_1 = -1, vf_2 = -1;
		if (vm.count("vf-aeff-1")) vf_aeff_1 = vm["vf-aeff-1"].as<double>();
		if (vm.count("vf-aeff-2")) vf_aeff_2 = vm["vf-aeff-2"].as<double>();
		if (vm.count("vf-val-1")) vf_1 = vm["vf-val-1"].as<double>();
		if (vm.count("vf-val-2")) vf_2 = vm["vf-val-2"].as<double>();
		//double vfo = vm["constant-vf-shape"].as<double>(); // Used only in one place
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
			double fv, maxDiamFull;
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
			VORONOI_OFFSET,
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
		else if (vfScaling == "Voronoi_Offset") vf = VFRAC_TYPE::VORONOI_OFFSET;
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
		else doHelp("Need to specify a proper refractive scaling.");

		string armeth = vm["ar-method"].as<string>(); // Used much further below


		auto process_commandline = [&]()
		{
			// If any of these are set, ensure that the temperature and frequency are also set
			mylog( "Processing command-line runs");
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
								mylog("Adding for aeff " << aeff << " vf " << vfrac << " temp " << temp);
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
								r.maxDiamFull = rtmath::units::convertLength(
									_in_length_value = aeff,
									_in_length_type = "Effective_Radius",
									_out_volume_fraction = vfrac,
									_ar = aspect,
									_out_length_type = "Max_Diameter");
								double mindim = convertLength(
									_in_length_value = aeff,
									_in_length_type = "Effective_Radius",
									_out_volume_fraction = vfrac,
									_ar = aspect,
									_out_length_type = "Min_Diameter"
									);
								mylog("\n\taeff " << aeff
									<< "\n\tmax dim " << r.maxDiamFull
									<< "\n\tmin dim " << mindim
									<< "\n\tvf " << vfrac);
								std::cerr << "aeff " << aeff
									<< " max dim " << r.maxDiamFull  << " min dim " << mindim
									<< " vf " << vfrac << std::endl;
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
									for (const auto &aeff1 : aeffs) {
										using namespace rtmath::density;
										double dIce = ice1h( _temperature = temp, _temp_units = "K" );
										double dEff = 0;
										dEff = effDen(
											_provider = vfScaling,
											_in_length_value = aeff1,
											_in_length_type = sizetype, //"Effective_Radius_Ice",
											_in_length_units = "um",
											_ar = aspect,
											_temperature = temp,
											_temp_units = "K",
											_vfLowAeff = vf_aeff_1,
											_vfHighAeff = vf_aeff_2,
											_vfLow = vf_1,
											_vfHigh = vf_2
											);
										double vf = dEff / dIce;
										double aeff = convertLength(
											_in_length_value = aeff1,
											_in_length_type = sizetype,
											_ar = aspect,
											_in_volume_fraction = vf,
											_out_length_type = "Effective_Radius"
											);
										mylog("auto vf"
											<< "\n\t_in_length_value " << aeff1
											<< "\n\t_in_length_type " << sizetype
											<< "\n\t_ar " << aspect
											<< "\n\t_out_length_type Effective_Radius"
											<< "\n\taeff " << aeff
											<< "\n\t_in_volume_fraction " << vf
											<< "\n\tvfScaling " << vfScaling
											<< "\n\ttemp " << temp << " K"
											<< "\n\teffDen " << dEff
											);
										aeff_vf_rad.push_back(std::tuple<double, double, double>
											(aeff, vf, temp));
									}
								}
							} else {
								if (!temps.size() && overrideM) temps.insert(-1);
								for (const auto &temp : temps)
									for (const auto &vf : vfracs) {
										for (const auto &aeff1 : aeffs) {
											using namespace rtmath::density;
											double dIce = ice1h( _temperature = temp, _temp_units = "K" );
											double vvf = vf;
											//if (vm.count("vf-is-den"))
											//	vvf /= dIce; // TODO: Not yet working.
											double aeff = convertLength(
												_in_length_value = aeff1,
												_in_length_type = sizetype,
												_ar = aspect,
												_in_volume_fraction = vvf,
												_out_length_type = "Effective_Radius"
												);
											mylog("manual vf"
												<< "\n\t_in_length_value " << aeff1
												<< "\n\t_in_length_type " << sizetype
												<< "\n\t_ar " << aspect
												<< "\n\t_in_volume_fraction " << vvf
												<< "\n\t_out_length_type Effective_Radius"
												<< "\n\taeff " << aeff);
											//std::cerr << "aeff " << inAeff << " rad " << rad << " vf " << vf << std::endl;
											aeff_vf_rad.push_back(std::tuple<double, double, double>
												(aeff, vvf, temp));
										}
								}
							}
							for (const auto &t : aeff_vf_rad)
								doAeff(std::get<0>(t), std::get<1>(t), std::get<2>(t));
						}
				}
				else doHelp("Need to specify temperatures or a refractive index");
			}
		};

		process_commandline();



		auto processShape = [&](boost::shared_ptr<rtmath::ddscat::shapefile::shapefile> s)
		{
			try {
				mylog("Processing shape " << s->hash().lower);
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
						r.ar = 1 / stats->calcStatsRot(0, 0, 0)->get<1>().at(rtmath::ddscat::stats::rotColDefs::AS_ABS)(0, 2);
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
					} else if ((vf == VFRAC_TYPE::VORONOI_OFFSET) && (int_voro_depth == 2)) {
						v = &(stats->SVoronoi_internal_2);
						r.fvMeth = "Internal Voronoi Offset Depth 2";
					} else if (vf == VFRAC_TYPE::OTHER) {
						r.fvMeth = vfScaling;
					}
					if (v) {
						// Cannot do this because effective radius != equivalent radius
						// r.aeff = v->aeff_V * s->standardD;
						// The stats aeff_V is a misnomer. Should be equivalent radius.
						// Instead, do this:
						//r.aeff = s->standardD
						//	* rtmath::units::convertLength(
						//		_in_length_value = v->V * v->f,
						//		_in_length_type = "Volume",
						//		_out_length_type = "Effective_Radius");
						// Stats implement aeff_V and V inconsistently. Voronoi is
						// different from the others. The reliable one is the volume
						// fraction. So, calculate everything from this.
						//double vol_real = stats->V_dipoles_const * pow(s->standardD,3.);
						//double vol_meth = vol_real / v->f;
						//r.aeff = pow(3.*v->V*v->f/(4.*pi),1./3.) * s->standardD;
						r.aeff = stats->aeff_dipoles_const * s->standardD;
						r.fv = v->f;
						// MAY HAVE CAUSED AN ERROR IN THE RESULTS!!!!!!!
						//if (vf == VFRAC_TYPE::VORONOI_OFFSET) r.fv -= 0.1;
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
						//if (vf == VFRAC_TYPE::VORONOI_OFFSET) r.fv -= 0.15;
					} else if (vf == VFRAC_TYPE::OTHER ) {
						using namespace rtmath::density;
						double dIce = ice1h( _temperature = temp, _temp_units = "K" );
						double dEff = 0;
						dEff = effDen(
							_provider = r.fvMeth,
							_in_length_value = stats->max_distance * s->standardD,
							_in_length_type = "Max_Diameter_Full",
							_in_length_units = "um",
							_ar = r.ar,
							_temperature = temp,
							_temp_units = "K",
//							_vfOverride = vfo
							_vfLowAeff = vf_aeff_1,
							_vfHighAeff = vf_aeff_2,
							_vfLow = vf_1,
							_vfHigh = vf_2
							);
						r.fv = dEff / dIce;
						r.aeff = stats->aeff_dipoles_const * s->standardD;
					}

					r.freq = freq;
					if (overrideM) r.m = ovM;
					else rtmath::refract::mIce(freq, temp, r.m);
					r.temp = temp;
					r.refHash = s->hash().string();
					r.lambda = rtmath::units::conv_spec("GHz", "um").convert(freq);
					r.maxDiamFull = stats->max_distance * s->standardD;
					if (vf == VFRAC_TYPE::VORONOI_OFFSET) {
						// maxDiamFull is in um. Want value to be the same at small diameters
						// and larger at large diameters. Max size is 11 mm. Min size ~1 mm.
						// At max size, want to reduce by ___.
						// 0.35 is great at 94 GHz AR 0.9 integrated bk.
						// Trying 0.33333.
						//std::cerr << "\t iv2 " << r.fv;
						r.fv *= ( 1. - (0.33333*(r.maxDiamFull / 10000)));
						//std::cerr << "\t vo2 " << r.fv << std::endl;
					}
					mylog("Adding shape run " << r.refHash <<
						"\n\taeff " << r.aeff << ", aeff_dipoles_const " << stats->aeff_dipoles_const <<
						"\n\tMax diam: " << r.maxDiamFull <<
						"\n\tfreq " << freq << "\n\ttemp " << temp <<
						"\n\tfv " << r.fv <<
						"\n\tar " << r.ar);
					std::cerr << "Max diam: " << r.maxDiamFull << ", standardD: " << s->standardD
						<< ", stats->max_distance: " << stats->max_distance << std::endl
						<< "\taeff " << r.aeff << ", aeff_dipoles_const: " << stats->aeff_dipoles_const << std::endl
						<< "\treference hash " << r.refHash << std::endl;

					runs.push_back(std::move(r));
					}
				} catch (Ryan_Debug::error::xMissingHash &err) {
					cerr << "  Cannot find hash for " << s->hash().lower << endl;
					cerr << err.what() << endl;
				}
		};

		for (const auto &s : *(res.first))
			processShape(rtmath::ddscat::shapefile::shapefile::generate(s));




		std::ofstream out(string(oprefix).c_str());
		// Output a header line
		out << "Cross-Section Method\tIce Aeff (um)\tMax Diameter (mm)\t"
			"Ice Volume (mm^3)\tFrequency (GHz)\t"
			"Volume Fraction\tEffective Density (g/cm^3)\t"
			"Temperature (K)\tHash\t"
			"AR Method\tRefractive Index Method\tVolume Fraction Method\t"
			"Aspect Ratio\tLambda (um)\tM_re\tM_im\t"
			"Size Parameter\t"
			"Theta\tBeta\tPhi\tg\t";
		if (vm.count("show-qs"))
			out << "Qabs\tQbk\tQext\tQsca" << std::endl;
		else out << "Cabs (m^2)\tCbk (m^2)\tCext (m^2)\tCsca (m^2)" << std::endl;

		std::cerr << "Doing " << runs.size() << " runs." << std::endl;
		if (vm.count("show-qs"))
			cout << "Meth\tg\tQabs (m^2)\tQbk (m^2)\tQext (m^2)\tQsca (m^2)" << std::endl;
		else
			cout << "Meth\tg\tCabs (m^2)\tCbk (m^2)\tCext (m^2)\tCsca (m^2)" << std::endl;
		size_t i=1;
		// Iterate over all possible runs
		mylog("Executing. There are " << runs.size() << " runs to process.");
		for (const auto &r : runs)
		{
			ostringstream ofiless;
			// the prefix is expected to be the discriminant for the rotational data
			ofiless << oprefix << "-t-" << r.temp << "-f-" << r.freq
				<< "-aeff-" << r.aeff << "-md-" << r.maxDiamFull
				<< "-vfrac-" << r.fv << "-aspect-" << r.ar;
			string ofile = ofiless.str();
			mylog(ofile << " --- " << i << " / " << runs.size() << " " << r.refHash);
			cout << ofile << " --- " << i << " / " << runs.size() << " " << r.refHash << endl;
			++i;

			const double sizep = 2. * pi * r.aeff / r.lambda;

			using namespace rtmath::phaseFuncs;
			pf_class_registry::inputParamsPartial i;
			i.aeff = r.aeff;
			i.aeff_rescale = vm["scale-aeff"].as<bool>();
			i.aeff_version = pf_class_registry::inputParamsPartial::aeff_version_type::EQUIV_V_SPHERE;
			i.eps = r.ar;
			i.m = r.m;
			i.ref = r.refHash;
			i.maxDiamFull = r.maxDiamFull;
			//std::cerr << "i.m = r.m = " << i.m << std::endl;
			i.rmeth = rmeth; // Yeah, only one refractive index method per program invocation is supported.
			i.shape = pf_class_registry::inputParamsPartial::shape_type::SPHEROID;
			i.vFrac = r.fv;
			i.other = otherOpts;

			mylog("run aeff " << i.aeff << "\n\taeff_rescale " << i.aeff_rescale <<
				"\n\taeff_version EQUIV_V_SPHERE\n\teps " << i.eps <<
				"\n\tm " << i.m.real() << ", " << i.m.imag() <<
				"\n\tref " << i.ref <<
				"\n\tmaxDiam " << i.maxDiamFull <<
				"\n\trmeth " << refractScaling <<
				"\n\tshape SPHEROID" <<
				"\n\tvFrac " << i.vFrac);

			if (r.fv <= 0 || r.fv > 1.) {
				std::cout << "\tCowardly refusing to calculate scattering for an invalid particle. See "
					"the log for details (channel=density). Skipping to next one." << std::endl;
				mylog("Error. r.fv is not in (0,1]. Skipping.");
				continue;
			}


			pf_provider p(i);

			pf_provider::resCtype res;
			pf_class_registry::setup s;
			s.beta = 0; s.theta = 0; s.phi = 0;
			s.sPhi = 0; s.sPhi0 = 0; s.sTheta = 0; s.sTheta0 = 0;
			s.wavelength = r.lambda;

			// smeth is parameter solution-method, which can force a single method to be used
			p.getCrossSections(s, res, smeth);


			double maxDiam = r.maxDiamFull, Vi = 0, effDen = 0; // V=0
			{
				using namespace rtmath::density;
				//double diammm = maxDiam / 1000.;
				//convertLength( _ar = r.ar,
				//	_in_length_units = "um",
				//	_out_length_units = "mm",
				//	_in_length_value = maxDiam,
				//	_in_length_type = "Max_Diameter_Full",
				//	_out_length_type = "Max_Diameter_Full"
				//	);
				//V = (4. * pi / 3.) * pow(diammm/2.,3.);
				Vi = (4. * pi / 3.) * pow(r.aeff/1000,3.);
				//Vi = V;
				//V /= r.fv;
				//double rad = pow(3.*V/(4.*pi),1./3.);
				//maxDiam = 2. * rad;

				double dIce = ice1h( _temperature = r.temp, _temp_units = "K" );
				effDen = dIce * r.fv;
			}

			for (const auto &rr : res)
			{
				double beta2 = s.beta, theta2 = s.theta, phi2 = s.phi;
				if (std::string(rr.first) == "SSRG") {
					if (otherOpts->hasVal("beta")) beta2 = otherOpts->getVal<double>("beta");
					if (otherOpts->hasVal("gamma")) theta2 = otherOpts->getVal<double>("gamma");
					if (otherOpts->hasVal("kappa")) phi2 = otherOpts->getVal<double>("kappa");
				}
				double Qabs = rr.second.Cabs / (pi * pow(r.aeff/1e6,2.));
				double Qsca = rr.second.Csca / (pi * pow(r.aeff/1e6,2.));
				double Qext = rr.second.Cext / (pi * pow(r.aeff/1e6,2.));
				double Qbk = rr.second.Cbk / (pi * pow(r.aeff/1e6,2.));
				if (rr.second.Cabs < 0 ) Qabs = -1;
				if (rr.second.Csca < 0 ) Qsca = -1;
				if (rr.second.Cext < 0 ) Qext = -1;
				if (vm.count("use-ddscat-conv")) Qbk /= 4.*pi;
				if (rr.second.Cbk < 0 ) Qbk = -1;
				out << rr.first << "\t" << r.aeff << "\t" << maxDiam / 1000. << "\t" 
					<< Vi << "\t"
					<< r.freq << "\t" << r.fv << "\t" << effDen << "\t" << r.temp << "\t"
					<< r.refHash << "\t" << armeth << "\t" << refractScaling << "\t" 
					<< r.fvMeth << "\t" //<< r.fvMeth * 0 << "\t" // get ice density here.....
					<< r.ar << "\t" << r.lambda << "\t" << r.m.real() << "\t" << r.m.imag() << "\t"
					<< sizep << "\t"
					<< theta2 << "\t" << beta2 << "\t" << phi2 << "\t"
					<< rr.second.g << "\t";
				if (vm.count("show-qs")) {
					out << Qabs << "\t" << Qbk << "\t" << Qext << "\t" << Qsca << std::endl;
				} else {
					out << rr.second.Cabs << "\t"
					<< rr.second.Cbk << "\t"
					<< rr.second.Cext << "\t"
					<< rr.second.Csca
					<< std::endl;
				}

				if (vm.count("show-qs")) {
					cout << rr.first << "\t" << rr.second.g << "\t"
						<< Qabs << "\t" << Qbk << "\t" << Qext << "\t" << Qsca << std::endl;
				} else {
				cout << rr.first << "\t"
					<< rr.second.g << "\t"
					<< rr.second.Cabs << "\t"
					<< rr.second.Cbk << "\t"
					<< rr.second.Cext << "\t"
					<< rr.second.Csca
					<< std::endl;
				}
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



