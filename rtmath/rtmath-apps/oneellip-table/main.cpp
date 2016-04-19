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
		cerr << "rtmath-oneellip-table" << endl;
		const double pi = boost::math::constants::pi<double>();
		// Do processing of argv
		namespace po = boost::program_options;
		po::options_description desc("Allowed options"), cmdline("Command-line options"),
			runmatch("Run-matching options"), basic("Basic ellipsoid options"),
			refract("Refractive index options"),
			config("Config options"), hidden("Hidden options"), oall("all options");

		rtmath::debug::add_options(cmdline, config, hidden);
		//rtmath::ddscat::stats::shapeFileStats::add_options(cmdline, config, hidden);
		//rtmath::ddscat::ddOutput::add_options(cmdline, config, hidden);
		//rtmath::refract::add_options(cmdline, config, hidden);

		runmatch.add_options()
			("input,i", po::value<string>(), "Specify input file")
			("col-md", po::value<int>()->default_value(3),
			 "Specify maximum dimension column")
			("col-aeff", po::value<int>()->default_value(2),
			 "Specify effective radius column")
			("col-ar", po::value<int>()->default_value(5),
			 "Specify aspect ratio column")
			("col-vf", po::value<int>()->default_value(9),
			 "Specify volume fraction column")
			("col-hash", po::value<int>()->default_value(1),
			 "Specify hash column")
			("tag", po::value<string>(), "Specify tag")
			("ar-method", po::value<string>()->default_value("Spheres"),
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
			("solution-method", po::value<std::string>()->default_value("bhmie"), "Force only a specific algorithm to be used, such as "
			 "Rayleigh or bhmie. No other pf generator will be used.")
			;
		basic.add_options()
			("freqs,f", po::value<std::string>(), "Specify frequencies in GHz. Needed for dielectrics.")
			("other-param,p", po::value<std::vector<std::string> >(), "Specify optional string parameters "
			 "that will be passed to the backend functions. Give in type:key:value notation. "
			 "Types are double or string.")
			("show-qs", "Instead of actual cross sections, show Q values")
			("use-ddscat-conv", "Use ddscat conventions for backscatter")
			;

		cmdline.add_options()
			("help,h", "produce help message")
			("output,o", po::value<string>(), "Set output file (required)")
			;

		desc.add(cmdline).add(config).add(runmatch).add(refract).add(basic);
		oall.add(cmdline).add(runmatch).add(refract).add(config).add(hidden).add(basic);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);
		//rtmath::ddscat::stats::shapeFileStats::process_static_options(vm);

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
		if (vm.count("output"))
			oprefix = vm["output"].as<string>();

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

		double dSpacing = 0;
		if (vm.count("dipole-spacing"))
			dSpacing = vm["dipole-spacing"].as<double>();

		struct run
		{
			double temp, freq, aeff, lambda, ar;
			double fv, maxDiamFull;
			std::string fvMeth;
			std::complex<double> m;
			std::string refHash;
			std::string tag;
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
		if (vm.count("mr") || vm.count("mi"))
		{
			overrideM = true;
			if (!vm.count("mr") || !vm.count("mi"))
				doHelp("When overriding refractive index, need to specify "
				"both real and imaginary components.");
			double r = vm["mr"].as<double>();
			double i = vm["mi"].as<double>();
			ovM = complex<double>(r, i);
		}
		auto fixedBinder = [&](std::complex<double>, std::complex<double>, double, std::complex<double> &Mres)
		{
			Mres = ovM;
		};

		string refractScaling = vm["refract-method"].as<string>();
		std::function<void(std::complex<double>, std::complex<double>, double, std::complex<double> &)> rmeth;
		if (refractScaling == "Maxwell_Garnett_Ellipsoids") rmeth = rtmath::refract::maxwellGarnettEllipsoids;
		else if (refractScaling == "Maxwell_Garnett_Spheres") rmeth = rtmath::refract::maxwellGarnettSpheres;
		else if (refractScaling == "Debye") rmeth = rtmath::refract::debyeDry;
		else if (refractScaling == "Bruggeman") rmeth = rtmath::refract::bruggeman;
		else if (!rescaleM) rmeth = fixedBinder;
		else doHelp("Need to specify a proper refractive scaling.");

		string armeth = vm["ar-method"].as<string>(); // Used much further below

		ofstream out(string(oprefix).c_str());
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
			i.aeff_rescale = true;
			i.aeff_version = pf_class_registry::inputParamsPartial::aeff_version_type::EQUIV_V_SPHERE;
			i.eps = r.ar;
			i.m = r.m;
			i.ref = r.refHash;
			i.maxDiamFull = r.maxDiamFull;
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
				Vi = (4. * pi / 3.) * pow(r.aeff/1000,3.);
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

