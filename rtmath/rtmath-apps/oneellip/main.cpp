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
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
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
			("scale-voronoi-internal", "Scale using the internal voronoi fraction")
			("voronoi-depth", po::value<size_t>()->default_value(2), "Sets the internal voronoi depth for scaling")
			("scale-circumscribing-sphere", "Scale using the circumscribing sphere fraction")
			("scale-convex", "Scale using the convex hull fraction")
			("scale-ellipsoid-max", "Scale using the max ellipsoid fraction")

			("ar-method", po::value<string>()->default_value("Max-Ellipsoids"),
			"Max-Ellipsoids: Force aspect ratios to be ellipsoids, following the max AR calculated in stats code. "
			"Spheres: Force aspect ratios to be spheres, instead of stats-determined spheroids. "
			"(TODO SA-V-Ellipsoids: Select AR of ellipsoid with matching surface area to volume ratio).")
			;
		refract.add_options()
			("method", po::value<string>()->default_value("Maxwell-Garnett-Ellipsoids"), "Method used to calculate the resulting dielectric "
			"(Sihvola, Debye, Maxwell-Garnett-Spheres, Maxwell-Garnett-Ellipsoids). "
			"Only matters if volume fractions are given. Then, default is Maxwell-Garnett-Ellipsoids.")
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
			std::string fvMeth;
			std::complex<double> m;
			std::string refHash;
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
		//bool scaleaeff = false;
		//if (vm.count("scale-aeff")) scaleaeff = true;


		enum class VFRAC_TYPE
		{
			CIRCUM_SPHERE,
			VORONOI,
			CONVEX,
			ELLIPSOID_MAX,
			INTERNAL_VORONOI
		};
		VFRAC_TYPE vf = VFRAC_TYPE::CIRCUM_SPHERE;
		if (vm.count("scale-circumscribing-sphere")) vf = VFRAC_TYPE::CIRCUM_SPHERE;
		else if (vm.count("scale-voronoi")) vf = VFRAC_TYPE::VORONOI;
		else if (vm.count("scale-voronoi-internal")) vf = VFRAC_TYPE::INTERNAL_VORONOI;
		else if (vm.count("scale-convex")) vf = VFRAC_TYPE::CONVEX;
		else if (vm.count("scale-ellipsoid-max")) vf = VFRAC_TYPE::ELLIPSOID_MAX;
		size_t int_voro_depth = vm["voronoi-depth"].as<size_t>();
		bool rescaleM = vm["scale-m"].as<bool>();

		string scaleMeth = vm["method"].as<string>();
		rtmath::phaseFuncs::pf_class_registry::inputParamsPartial::refract_method rmeth = 
			rtmath::phaseFuncs::pf_class_registry::inputParamsPartial::refract_method::NONE;
		if (scaleMeth == "Sihvola") rmeth =
			rtmath::phaseFuncs::pf_class_registry::inputParamsPartial::refract_method::SIHVOLA;
		else if (scaleMeth == "Maxwell-Garnett-Ellipsoids") rmeth =
			rtmath::phaseFuncs::pf_class_registry::inputParamsPartial::refract_method::MG_ELLIPSOIDS;
		else if (scaleMeth == "Maxwell-Garnett-Spheres") rmeth =
			rtmath::phaseFuncs::pf_class_registry::inputParamsPartial::refract_method::MG_SPHERES;
		else if (scaleMeth == "Debye") rmeth =
			rtmath::phaseFuncs::pf_class_registry::inputParamsPartial::refract_method::DEBYE;
		
		string armeth = vm["ar-method"].as<string>();

		using rtmath::ddscat::ddOutput;
		auto process_indiv_ddoutput = [&](boost::shared_ptr<ddOutput> s)
		{
			run r;
			s->loadShape(true);
			rtmath::ddscat::stats::shapeFileStatsBase::volumetric *v = nullptr;
			if (vf == VFRAC_TYPE::CIRCUM_SPHERE) {
				v = &(s->stats->Scircum_sphere); r.fvMeth = "Circumscribing Sphere";
			} else if (vf == VFRAC_TYPE::VORONOI) {
				v = &(s->stats->SVoronoi_hull); r.fvMeth = "Voronoi hull";
			} else if (vf == VFRAC_TYPE::CONVEX) {
				v = &(s->stats->Sconvex_hull); r.fvMeth = "Convex hull";
			} else if (vf == VFRAC_TYPE::ELLIPSOID_MAX) {
				v = &(s->stats->Sellipsoid_max);
				r.fvMeth = "Max ellipsoid";
			}
			if (v) {
				r.aeff = v->aeff_V;
				r.fv = v->f;
			} else if (vf == VFRAC_TYPE::INTERNAL_VORONOI) {
				auto shp = s->shape; // shape and stats loaded above
				boost::shared_ptr<rtmath::Voronoi::VoronoiDiagram> vd;
				vd = shp->generateVoronoi(
					std::string("standard"), rtmath::Voronoi::VoronoiDiagram::generateStandard);
				vd->calcSurfaceDepth();
				vd->calcCandidateConvexHullPoints();

				size_t numLatticeTotal = 0, numLatticeFilled = 0;
				vd->calcFv(int_voro_depth, numLatticeTotal, numLatticeFilled);

				r.aeff = s->aeff; /// TODO: CHECK ACCURACY AND CONSISTENCY OF AEFF DEFINITIONS!
				r.fvMeth = "Internal Voronoi Depth ";
				r.fvMeth.append(boost::lexical_cast<std::string>(int_voro_depth));
				r.fv = (double)numLatticeFilled / (double)numLatticeTotal;
			}
			else RTthrow rtmath::debug::xBadInput("Unhandled volume fraction method");
			
			/// Gives aspect ratio of matching ellipsoid
			/// \todo Implement this function
			auto arEllipsoid = [](double sa, double v) -> double
			{
				RTthrow rtmath::debug::xUnimplementedFunction();
				return -1;
			};

			
			if (armeth == "Max-Ellipsoids")
				r.ar = s->stats->calcStatsRot(0, 0, 0)->get<1>().at(rtmath::ddscat::stats::rotColDefs::AS_ABS)(0, 1);
			else if (armeth == "Spheres")
				r.ar = 1;
			else doHelp("ar-method needs a correct value.");
			r.freq = s->freq;
			r.m = (overrideM) ? ovM : s->ms.at(0).at(0);
			r.temp = s->temp;
			r.refHash = s->shapeHash.string();
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
					ddOutput::readVector(nullptr, iopts, dos, nullptr);
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
								rtmath::refract::mIce(freq, temp, r.m);
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

		if (vm.count("ddoutput")) vddoutputs = vm["ddoutput"].as<vector<string> >();
		process_ddoutput();
		process_commandline();
		
		


		ofstream out( string(oprefix).append(".tsv").c_str());
		// Output a header line
		out << "Method\tAeff (um)\tFrequency (GHz)\tVolume Fraction\tTemperature (K)\tHash\tAR Meth\tScale Meth\t"
			"Aspect Ratio\tLambda (um)\tM_re\tM_im\t"
			"Size Parameter\tRescale aeff\tfv Method\t"
			"Theta\tBeta\tPhi\tg\tg_iso\tQabs\tQabs_iso\tQbk\tQbk_iso\tQext\tQext_iso\tQsca\tQsca_iso" << std::endl;

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
			i.aeff_rescale = vm["scale-aeff"].as<bool>();
			i.aeff_version = pf_class_registry::inputParamsPartial::aeff_version_type::EQUIV_V_SPHERE;
			i.eps = r.ar;
			i.m = r.m;
			
			i.m_rescale = rmeth;
			if (!rescaleM) i.m_rescale = pf_class_registry::inputParamsPartial::refract_method::NONE;
			i.shape = pf_class_registry::inputParamsPartial::shape_type::SPHEROID;
			i.vFrac = r.fv;


			pf_provider p(o, i);

			pf_provider::resCtype res;
			pf_class_registry::setup s;
			s.beta = 0; s.theta = 0; s.phi = 0;
			s.sPhi = 0; s.sPhi0 = 0; s.sTheta = 0; s.sTheta0 = 0;
			s.wavelength = r.lambda;
			p.getCrossSections(s, res);

			for (const auto &rr : res)
			{
				out << rr.first << "\t" << r.aeff << "\t" << r.freq << "\t" << r.fv << "\t" << r.temp << "\t" 
					<< r.refHash << "\t" << armeth << "\t" << scaleMeth << "\t"
					<< r.ar << "\t" << r.lambda << "\t" << r.m.real() << "\t" << r.m.imag() << "\t"
					<< sizep  << "\t" << i.aeff_rescale << "\t" << r.fvMeth << "\t"
					<< s.theta << "\t" << s.beta << "\t" << s.phi << "\t" 
					<< rr.second.g << "\t" << rr.second.g_iso << "\t"
					<< rr.second.Qabs << "\t" << rr.second.Qabs_iso << "\t" << rr.second.Qbk << "\t"
					<< rr.second.Qbk_iso << "\t" << rr.second.Qext << "\t" << rr.second.Qext_iso << "\t"
					<< rr.second.Qsca << "\t" << rr.second.Qsca_iso
					<< std::endl;
			}
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



