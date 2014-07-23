#include <iostream>
#include <boost/program_options.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/random/random_device.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_on_sphere.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <fstream>
#include <complex>
#include <set>
#include <vector>
#include <sstream>
#include <string>
#include <Ryan_Debug/debug.h>
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/phaseFunc.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/ddscat/ddOriData.h"
#include "../../rtmath/rtmath/ddscat/ddUtil.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

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
		rtmath::ddscat::ddOutput::add_options(cmdline, config, hidden);
		//rtmath::refract::add_options(cmdline, config, hidden);

		runmatch.add_options()
			("ddoutput", po::value<vector<string> >(), "Specify ddscat output to use when regenerating")
			("scale-voronoi", "Scale using the voronoi fraction")
			("scale-circumscribing-sphere", "Scale using the circumscribing sphere fraction")
			("scale-convex", "Scale using the convex hull fraction")
			("scale-ellipsoid-max", "Scale using the max ellipsoid fraction")
			;
		refract.add_options()
			("temps,T", po::value<std::string>()->default_value("263"), "Specify temperatures in K")
			("mr", po::value<double>(), "Override real refractive index value")
			("mi", po::value<double>(), "Override imaginary refractive index value")
			("volume-fractions,v", po::value<std::string>()->default_value("1"), "Set the ice volume fractions")
			("nu,n", po::value<double>()->default_value(0.85), "Value of nu for Sihvola refractive index scaling")
			;
		basic.add_options()
			("aspect-ratios,s", po::value<std::string>()->default_value("1"), "Specify aspect ratio for ellipsoids.")
			("aeffs,a", po::value<std::string>(), "Specify the effective radii in um")
			("freqs,f", po::value<std::string>(), "Specify frequencies in GHz. Needed for dielectrics.")
			;
		scale.add_options()
			("scale-aeff", "Scale effective radius based on volume fraction")
			("scale-m", "Scale refractive index based on volume fraction")
			("scale-v", "Scale for an equivalent volume ellipsoid")
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
		rtmath::ddscat::ddUtil::process_static_options(vm);
		rtmath::ddscat::stats::shapeFileStats::process_static_options(vm);
		rtmath::ddscat::ddOutput::process_static_options(vm);
		//rtmath::refract::process_static_options(vm);

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

		struct run
		{
			double temp, freq, aeff, lambda, ar;
			double fv;
			std::complex<double> m;
		};
		vector<run> runs;
		runs.reserve(50000);

		// First, set any overrides for the frequency, temperature and refractive index
		set<double> temps, freqs;
		if (vm.count("temps"))
			rtmath::config::splitSet(vm["temps"].as<string>(), temps);
		if (vm.count("freqs"))
			rtmath::config::splitSet(vm["freqs"].as<string>(), freqs);
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

		//double vfrac = vm["volume-fraction"].as<double>();
		bool scale = false;
		if (vm.count("scale-aeff")) scale = true;

		using rtmath::ddscat::ddOutput;
		auto process_indiv_ddoutput = [&](boost::shared_ptr<ddOutput> s)
		{
			run r;
			r.aeff = aeff;
			r.ar = aspect;
			r.freq = freq;
			r.fv = vfrac;
			r.lambda = rtmath::units::conv_spec("GHz", "um").convert(freq);
			r.m = ovM;
			r.temp = -1;
			runs.push_back(std::move(r));
		};

		vector<string> vddoutputs;
		auto process_ddoutput = [&]()
		{
			using namespace boost::filesystem;
			for (const auto &i : vddoutputs)
			{
				cerr << "Processing " << i << endl;
				path ps = rtmath::debug::expandSymlink(i);

				auto iopts = rtmath::registry::IO_options::generate();
				iopts->filename(i);
				// Handle not needed as the read context is used only once.
				if (is_directory(ps))
				{
					// Check for recursion
					path pd = ps / "shape.dat";
					if (!exists(pd))
					{
						// Load one level of subdirectories
						vector<path> subdirs;
						copy(directory_iterator(ps),
							directory_iterator(), back_inserter(subdirs));
						for (const auto &p : subdirs)
						{
							if (is_directory(p))
							{
								cerr << " processing " << p << endl;
								boost::shared_ptr<ddOutput> s(new ddOutput);
								s = ddOutput::generate(ps.string());
								process_indiv_ddoutput(s);
							}
						}
					}
					else {
						// Input is a ddscat run
						boost::shared_ptr<ddOutput> s(new ddOutput);
						s = ddOutput::generate(ps.string());
						process_indiv_ddoutput(s);
					}
				}
				else if (ddOutput::canReadMulti(nullptr, iopts))
				{
					vector<boost::shared_ptr<ddOutput> > dos;
					ddOutput::readVector(nullptr, iopts, dos);
					for (const auto &s : dos)
						process_indiv_ddoutput(s);
				} else {
					// This fallback shouldn't happen...
					boost::shared_ptr<ddOutput> s(new ddOutput);
					s->readFile(i);
					process_indiv_ddoutput(s);
				}

			}

		};

		auto process_commandline = [&]()
		{
			// If any of these are set, ensure that the temperature and frequency are also set
			
			set<double> aeffs, aspects, vfracs; // , alphas, betas;
			if (vm.count("aeffs"))
				rtmath::config::splitSet(vm["aeffs"].as<string>(), aeffs);
			if (vm.count("aspect-ratios"))
				rtmath::config::splitSet(vm["aspect-ratios"].as<string>(), aspects);
			if (vm.count("volume-fractions"))
				rtmath::config::splitSet(vm["volume-fractions"].as<string>(), vfracs);

			if (freqs.size() && aeffs.size() && aspects.size() && vfracs.size())
			{
				if (temps.size() || overrideM)
				{
					for (const auto &freq : freqs)
					for (const auto &aeff : aeffs)
						for (const auto &aspect : aspects)
							for (const auto &vfrac : vfracs)
							{
						if (overrideM)
						{
							run r;
							r.aeff = aeff;
							r.ar = aspect;
							r.freq = freq;
							r.fv = vfrac;
							r.lambda = rtmath::units::conv_spec("GHz", "um").convert(freq);
							r.m = ovM;
							r.temp = -1;
							runs.push_back(std::move(r));
						} else {
							for (const auto &temp : temps)
							{
								run r;
								r.aeff = aeff;
								r.ar = aspect;
								r.freq = freq;
								r.fv = vfrac;
								r.lambda = rtmath::units::conv_spec("GHz", "um").convert(freq);
								r.temp = temp;
								runs.push_back(std::move(r));
							}
						}
							}
				}
				else doHelp("Need to specify temperatures or a refractive index");
			}
			else if (aeffs.size() || aspects.size() || vfracs.size())
			{
				std::ostringstream o;
				o << "Need to specify: ";
				if (!aeffs.size()) o << "aeffs, ";
				if (!aspects.size()) o << "aspect ratios, ";
				if (!vfracs.size()) o << "volume fractions, ";
				if (!freqs.size()) o << "frequencies, ";
				if (!overrideM && !temps.size()) o << "temperatures or dielectric overloads";
				doHelp(o.str());
			}

		};

		vector<string> vddoutputs;
		if (vm.count("ddoutput")) vddoutputs = vm["ddoutput"].as<vector<string> >();
		process_ddoutput();
		process_commandline();
		
		


		ofstream out( string(oprefix).append(".tsv").c_str());
		// Output a header line

		// Iterate over all possible runs
		for (const auto &r : runs)
		{
			ostringstream ofiless;
			// the prefix is expected to be the discriminant for the rotational data
			ofiless << oprefix << "-t-" << r.temp << "-f-" << r.freq 
				<< "-aeff-" << r.aeff 
				<< "-vfrac-" << r.fv << "-aspect-" << r.ar;
			string ofile = ofiless.str();
			cout << ofile << endl;


			const double sizep = 2. * boost::math::constants::pi<double>() * r.aeff / r.lambda;
			
			using namespace rtmath::phaseFuncs;
			pf_class_registry::orientation_type o = pf_class_registry::orientation_type::ISOTROPIC;
			pf_class_registry::inputParamsPartial i;
			i.aeff = r.aeff;
			i.aeff_rescale;
			i.aeff_version = pf_class_registry::inputParamsPartial::aeff_version_type::EQUIV_V_SPHERE;
			i.eps = r.ar;
			i.m;
			i.m_rescale;
			i.shape = pf_class_registry::inputParamsPartial::shape_type::SPHEROID;
			i.vFrac = r.fv;

			pf_provider p(o, i);

			pf_provider::resCtype res;
			pf_class_registry::setup s;
			p.getCrossSections()
		}

	} catch (std::exception &e) {
		cerr << "Exception: " << e.what() << endl;
		exit(2);
	} catch (...) {
		cerr << "Caught unidentified error... Terminating." << endl;
		exit(1);
	}

	return 0;
}



