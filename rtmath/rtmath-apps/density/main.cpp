/// Density program tablulates various particle size / density relations.
/// It accounts for temperature and different length scale units.
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
	try {
		auto& lg = m_app::get();
		cerr << "rtmath-density" << endl;
		// Do processing of argv
		namespace po = boost::program_options;
		po::options_description desc("Allowed options"), cmdline("Command-line options"),
			runmatch("Run-matching options"), basic("Basic ellipsoid options"),
			refract("Density options"),
			config("Config options"), hidden("Hidden options"), oall("all options");

		rtmath::debug::add_options(cmdline, config, hidden);

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
			("size-type", po::value<std::string>()->default_value("Effective_Radius_Ice"), "How is the size described? "
			 "Effective_Radius_Ice, Effective_Radius_Full, Max_Diameter_Full, etc. are valid options.")
			("vf-aeff-1", po::value<double>(), "")
			("vf-aeff-2", po::value<double>(), "")
			("vf-val-1", po::value<double>(), "")
			("vf-val-2", po::value<double>(), "")
			;

		cmdline.add_options()
			("help,h", "produce help message")
			("output-prefix,o", po::value<string>()->default_value("output"), "Set output file (required)")
			;

		desc.add(cmdline).add(config).add(runmatch).add(refract).add(basic);
		oall.add(cmdline).add(runmatch).add(refract).add(config).add(hidden).add(basic);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);
		//rtmath::ddscat::stats::shapeFileStats::process_static_options(vm);

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

		double vf_aeff_1 = -1, vf_aeff_2 = -1, vf_1 = -1, vf_2 = -1;
		if (vm.count("vf-aeff-1")) vf_aeff_1 = vm["vf-aeff-1"].as<double>();
		if (vm.count("vf-aeff-2")) vf_aeff_2 = vm["vf-aeff-2"].as<double>();
		if (vm.count("vf-val-1")) vf_1 = vm["vf-val-1"].as<double>();
		if (vm.count("vf-val-2")) vf_2 = vm["vf-val-2"].as<double>();
		struct run
		{
			double temp, aeff, ar, md;
			double fv, dens;
			std::string fvMeth;
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
			string sizetype = vm["size-type"].as<string>();
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

			using namespace rtmath::units::keywords;
			if (!temps.size() ) doHelp("Need to specify temperatures.");
			if ((aeffs.size() || radii.size()) && aspects.size() && vfracs.size())
			{
				for (const auto &aspect : aspects)
				for (const auto &vfrac : vfracs)
				{
					auto doAeff = [&](double aeff, double vfrac, double temp)
					{
						mylog("Adding for aeff " << aeff << " vf " << vfrac << " temp " << temp);
						run r;
						r.aeff = aeff;
						r.ar = aspect;
						r.fv = vfrac;
						r.temp = temp;
						if (vf == VFRAC_TYPE::OTHER) {
							r.fvMeth = vfScaling;
						}

						r.md = rtmath::units::convertLength(
							_in_length_value = aeff,
							_in_length_type = "Effective_Radius",
							_out_volume_fraction = vfrac,
							_ar = aspect,
							_out_length_type = "Max_Diameter");
						double mindim = rtmath::units::convertLength(
							_in_length_value = aeff,
							_in_length_type = "Effective_Radius",
							_out_volume_fraction = vfrac,
							_ar = aspect,
							_out_length_type = "Min_Diameter"
							);
						mylog("\n\taeff " << aeff
							<< "\n\tmax dim " << r.md
							<< "\n\tmin dim " << mindim
							<< "\n\tvf " << vfrac);
						std::cerr << "aeff " << aeff
							<< " max dim " << r.md  << " min dim " << mindim
							<< " vf " << vfrac << std::endl;
						runs.push_back(std::move(r));
					};
					std::vector<std::tuple<double, double, double> > aeff_vf_rad;
					// Density is temperature-dependent.
					if (!vfracs.size() && vf == VFRAC_TYPE::OTHER)
					{
						if (!temps.size()) doHelp("When using a predetermined density "
							"formula, temperature should be specified.");
						for (const auto &temp : temps) {
						// Populate the volume fractions being calculated
							for (const auto &aeff1 : aeffs) {
								using namespace rtmath::density;
								double dIce = ice1h(
									_temperature = temp, _temp_units = "K" );
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
								double aeff = rtmath::units::convertLength(
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
						if (!temps.size() ) temps.insert(-1);
						for (const auto &temp : temps)
							for (const auto &vf : vfracs) {
								for (const auto &aeff1 : aeffs) {
									using namespace rtmath::density;
									double dIce = ice1h( _temperature = temp, _temp_units = "K" );
									double vvf = vf;
									//if (vm.count("vf-is-den"))
									//	vvf /= dIce; // TODO: Not yet working.
									double aeff = rtmath::units::convertLength(
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
		};

		process_commandline();




		ofstream out(string(oprefix).c_str());
		// Output a header line
		out << "Aeff (um)\tMax_Diameter (um)\t"
			"Volume Fraction\tDensity (g/cm^3)\t"
			"Temperature (K)\tVolume Fraction Method\t"
			"Aspect Ratio"
			<< std::endl;

		std::cerr << "Doing " << runs.size() << " calculations." << std::endl;
		size_t i=0;
		// Iterate over all possible runs
		for (const auto &r : runs)
		{
			ostringstream ofiless;
			// the prefix is expected to be the discriminant for the rotational data
			ofiless << oprefix << "-t-" << r.temp 
				<< "-aeff-" << r.aeff
				<< "-vfrac-" << r.fv << "-aspect-" << r.ar;
			string ofile = ofiless.str();
			cout << ofile << " --- " << i << endl;
			++i;

				out << r.aeff << "\t"
					<< r.md << "\t"
					<< r.fv << "\t"
					<< r.dens << "\t"
					<< r.temp << "\t"
					<< r.fvMeth << "\t"
					<< r.ar
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

