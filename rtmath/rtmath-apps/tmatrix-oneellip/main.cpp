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
#include <tmatrix/tmatrix.h>
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/mie/mie.h"
#include "../../rtmath/rtmath/mie/mie-Qcalc.h"
#include "../../rtmath/rtmath/units.h"

int main(int argc, char *argv[])
{
	using namespace std;
	try {
		cerr << "rtmath-tmatrix-oneellip" << endl;
		// Do processing of argv
		namespace po = boost::program_options;
		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("aspect-ratios,s", po::value<std::string>()->default_value("1"), "Specify aspect ratio for ellipsoids.")
			("aeffs,a", po::value<std::string>(), "Specify the effective radii in um")
			("temps,T", po::value<std::string>()->default_value("263"), "Specify temperatures in K")
			("freqs,f", po::value<std::string>(), "Specify frequencies in GHz. Needed for dielectrics.")
			("output-prefix,o", po::value<string>()->default_value("output"), "Set output file (required)")
			("volume-fractions,v", po::value<std::string>()->default_value("1"), "Set the ice volume fractions")
			("nu,n", po::value<double>()->default_value(0.85), "Value of nu for Sihvola refractive index scaling")
			("scale-aeff", "Scale effective radius based on volume fraction")
			("mr", po::value<double>(), "Override real refractive index value")
			("mi", po::value<double>(), "Override imaginary refractive index value")
			("alphas", po::value<string>()->default_value("0"), "Set first rotation for backscatter calculation. UNDER DEVELOPMENT.")
			("betas", po::value<string>()->default_value("0"), "Set second rotation for backscatter calculation. UNDER DEVELOPMENT.")
			("random-rotations,r", po::value<size_t>(),
			"Replaces standard alpha and beta angles with random points.")
		;

		po::variables_map vm;
		po::store(po::command_line_parser(argc,argv).options(desc).run(), vm);
		po::notify(vm);

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

		if (!vm.count("freqs")) doHelp("Need to specify frequencies.");
		if (!vm.count("aeffs")) doHelp("Need to specify effective radii.");

		//double vfrac = vm["volume-fraction"].as<double>();
		bool scale = false;
		if (vm.count("scale-aeff")) scale = true;

		double nu = vm["nu"].as<double>();

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
			ovM = complex<double>(r,i);
			nu = -1.0;
		}

		set<double> temps, freqs, aeffs, aspects, vfracs, alphas, betas;
		rtmath::config::splitSet(vm["temps"].as<string>(), temps);
		rtmath::config::splitSet(vm["freqs"].as<string>(), freqs);
		rtmath::config::splitSet(vm["aeffs"].as<string>(), aeffs);
		rtmath::config::splitSet(vm["aspect-ratios"].as<string>(), aspects);
		rtmath::config::splitSet(vm["volume-fractions"].as<string>(), vfracs);
		rtmath::config::splitSet(vm["alphas"].as<string>(), alphas);
		rtmath::config::splitSet(vm["betas"].as<string>(), betas);

		// Precalculate the standard angle pairs
		std::vector<std::pair<double, double> > angles_pre;
		angles_pre.reserve(alphas.size() * betas.size());
		for (const double &alpha : alphas)
			for (const double &beta : betas)
			{
				double ra = alpha;
				if (ra >= 179.9) ra = 179.99; /// \todo Fix the tmatrix code here.
				angles_pre.push_back(std::pair<double,double>(ra,beta));
			}

		// The actual runs begin here
		using namespace tmatrix;
		//using namespace rtmath::mie;
		using namespace std;
		
		const double pi = boost::math::constants::pi<double>();
		typedef boost::random::mt19937 gen_type;
		gen_type rand_gen;
		rand_gen.seed(static_cast<unsigned int>(std::time(0)));
		boost::random::uniform_on_sphere<double> dist(3);
		boost::variate_generator<gen_type&, boost::uniform_on_sphere<double> >
			random_on_sphere(rand_gen, dist);
		boost::random::uniform_real_distribution<double> distUni(0,2.*pi);


		ofstream out( string(oprefix).append(".tsv").c_str());
		out << "Temperature (K)\tFrequency (GHz)\tEffective Radius (um)\tAspect Ratio\t"
			"mrr\tmri\tVolume Fraction\tnu\tSize Parameter\tAlpha (degrees)\tBeta (degrees)\tQabs_iso\tQsca_iso\tQext_iso\tQsca\tQbk" << endl;

		for (const double &temp : temps)
		for (const double &freq : freqs)
		for (const double &aeff : aeffs)
		for (const double &vfrac : vfracs)
		for (const double &aspect : aspects)
		{
			ostringstream ofiless;
			// the prefix is expected to be the discriminant for the rotational data
			ofiless << oprefix << "-t-" << temp << "-f-" << freq 
				<< "-aeff-" << aeff << "-nu-" << nu 
				<< "-vfrac-" << vfrac << "-aspect-" << aspect;
			string ofile = ofiless.str();
			cout << ofile << endl;

			complex<double> mIce, m, mAir(1,0);
			rtmath::refract::mIce(freq, temp, mIce);
			if (overrideM)
			{
				m = ovM;
			} else {
				rtmath::refract::sihvola(mIce,mAir,vfrac,nu,m);
			}

			double saeff = aeff;
			if (scale)
			{
				double sV = pow(aeff,3.0);
				sV /= vfrac;
				saeff = pow(sV,1./3.);
			}

			double lam = rtmath::units::conv_spec("GHz","um").convert(freq);
			const double sizep = 2. * boost::math::constants::pi<double>() * aeff / lam;
			
			

			tmatrixBase base;
			base.AXI = saeff;
			base.LAM = lam;
			base.MRR = m.real();
			base.MRI = abs(m.imag());
			base.EPS = aspect;
			if ( abs(aspect - 1.0) < 0.00001 ) base.EPS = 1.0001;
			const double k = 2. * pi / lam;
			boost::shared_ptr<const tmatrixParams> params = tmatrixParams::create(base);

			const double nRots = (double) alphas.size() * (double) betas.size();

			vector<std::pair<double,double> > angles;
			if (!vm.count("random-rotations"))
			{
				angles = angles_pre;
			} else {
				size_t n = vm["random-rotations"].as<size_t>();
				angles.reserve(n);

				for (size_t i=0; i<n; ++i)
				{
					// Convert from cartesian coordinates to rotation angles (in degrees)
					// The norm of the input vector is unity.
					auto radToDeg = [&pi](double rad) -> double
					{
						double res = rad * 180 / pi;
						return res;
					};

					vector<double> crdsCartesian = random_on_sphere();

					//float beta_R = distUni(rand_gen);
					//float &r = beta_R;
			
					double theta_R = acos(crdsCartesian[2]);
					double phi_R = atan2(crdsCartesian[1], crdsCartesian[0]) + pi;

					double theta_D = radToDeg(theta_R);
					double phi_D = radToDeg(phi_R) / 2.;

					angles.push_back(std::pair<double,double>(theta_D,phi_D));
				}
			}

			for (const auto &angle : angles)
			{
				const double &alpha = angle.first;
				const double &beta = angle.second;
				auto res = OriTmatrix::calc(params, alpha, beta);
				auto ang = OriAngleRes::calc(res, 0, 0, 180, 0); // theta = 0, phi = 90
				double C_sphere = pi * pow(saeff,2.0);
				double Qbk = getDifferentialBackscatterCrossSectionUnpol(res);
				double Qext_iso = res->qext;
				double Qsca_iso = res->qsca;
				
				double p = saeff / 2.;

				double Qsca = 8. * pi / (3. * k * k) * ang->getP(0,0) / C_sphere / C_sphere; // at theta = 0, phi = pi / 2.
				// unpolarized version for generalized ellipsoid. see yurkin 2013.
				//double Qext = 0;
				//double g = 0;
				double Qabs_iso = Qext_iso - Qsca_iso;
				//double Qabs = Qext - Qsca;

				out << temp << "\t" << freq << "\t" << aeff << "\t" << aspect << "\t"
				<< m.real() << "\t" << abs(m.imag()) << "\t" << vfrac << "\t" 
				<< nu << "\t" << sizep << "\t" << alpha << "\t" << beta << "\t"
				<< Qabs_iso << "\t" << Qsca_iso << "\t" << Qext_iso << "\t"
				<< Qsca << "\t" << Qbk << endl;
			}
		}

		return 0;
	} 
	catch (std::exception &e)
	{
		cerr << "Exception: " << e.what() << endl;
		exit(2);
	}
	catch (...)
	{
		cerr << "Caught unidentified error... Terminating." << endl;
		exit(1);
	}
}



