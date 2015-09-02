// This program takes the csv version of the stata output and
// performs a psd integration of backscatter and scattering
// cross-sections. It is given the aeff_ice column and a range
// of columns to integrate over, and it performs the integration
// using the Sekhon and Srivastava [1970] size distribution.

#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/spirit/include/karma.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
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
		cerr << "rtmath-integ-paper-1" << endl;
		const double pi = boost::math::constants::pi<double>();
		// Do processing of argv
		namespace po = boost::program_options;
		po::options_description desc("Allowed options"), cmdline("Command-line options"),
			config("Config options"), hidden("Hidden options"), oall("all options");

		rtmath::debug::add_options(cmdline, config, hidden);
		rtmath::ddscat::stats::shapeFileStats::add_options(cmdline, config, hidden);
		//rtmath::ddscat::ddOutput::add_options(cmdline, config, hidden);
		//rtmath::refract::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("input,i", po::value<string>(), "The input file (csv format)")
			("range", po::value<string>(), "The set of the columns to be integrated over")
			("x", po::value<size_t>(), "The x column (aeff_water_um). Starts at 1.")
			("keep,k", po::value<string>(), "Columns to keep")
			("tag,t", po::value<string>(), "An identifying tag column (optional)")
			("output,o", po::value<string>(), "Output file")
			("r", po::value<string>(), "The rainfall rates to consider (in mm/hr)")
			("help,h", "produce help message")
			;

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);

		auto& lg = m_app::get();
		// Begin checking parameters
		auto doHelp = [&](const std::string& s)
		{
			cout << s << endl;
			cout << desc << endl;
			exit(3);
		};

		if (vm.count("help") || vm.size() == 0) doHelp("");



		ofstream out(string(oprefix).c_str());
		// Output a header line
		out << "Cross-Section Method\tIce Aeff (um)\tMax Diameter (mm)\t"
			"Ice Volume (mm^3)\tFrequency (GHz)\t"
			"Volume Fraction\tEffective Density (g/cm^3)\t"
			"Temperature (K)\tHash\t"
			"AR Method\tRefractive Index Method\tVolume Fraction Method\t"
			"Aspect Ratio\tLambda (um)\tM_re\tM_im\t"
			"Size Parameter\tRescale aeff\t"
			"Theta\tBeta\tPhi\tg\tQabs\tQbk\tQext\tQsca" << std::endl;

		std::cerr << "Doing " << runs.size() << " runs." << std::endl;
		cout << "Meth\tg\tQabs\tQbk\tQext\tQsca" << std::endl;
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
			pf_class_registry::orientation_type o = pf_class_registry::orientation_type::ISOTROPIC;
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


			pf_provider p(o, i);

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
				out << rr.first << "\t" << r.aeff << "\t" << maxDiam / 1000. << "\t" 
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



