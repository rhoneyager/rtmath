// this is a drop-in replacement for Liu's getm and genmtab
// It will add extra functionality, such as a better selection of refractive index
// calculation functions. It will also be able to handle both ice and water.

#include <iostream>
#include <fstream>
#include <string>
#include <set>
#include <vector>
#include <map>
//#include "../../rtmath/rtmath/"
#include "../../rtmath/rtmath/error/error.h"
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/command.h"

#include <memory>
#include <complex>
#include <cmath>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/tuple/tuple.hpp>

void write(const std::complex<double> &ref, bool mtab);

enum PHASE {
	ICE,
	WATER,
	AIR
};

int main(int argc, char** argv)
{
	using namespace std;
	try {
		rtmath::debug::appEntry(argc, argv);
		bool mtab = false;

		// detect if rtmath-mtab is being called

		namespace po = boost::program_options;
		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("output,o", "Write ROOT output table of results")
			("mtab,m", "Produce mtab-style output")
			("frequency,f", po::value<string>(), "List of frequencies (GHz)")
			("temperature,T", po::value<string>(), "List of temperatures (K)")
			("volume-fraction-ice,i", po::value<string >(), "Volume fractions [0,1] for mixed phase volumes. If not given, calculate from others.")
			("volume-fraction-water,w", po::value<string>(), "Water volume fractions. If not given, calculation depends on presence of both ice and water fractions. If both are given, w = 1 - a - i. If only one is given, assume that water fraction is zero.")
			("volume-fraction-air,a", po::value<string>(), "Air volume fraction. If not given, calculate from others.")
			("nu", po::value<string>()->default_value("0.85"), "Value of nu for Sihvola (default is 0.85. Range is [0,2].")
			("method", po::value<string>()->default_value("Sihvola"), "Method used to calculate the resulting dielectric (Sihvola, Debye, Maxwell-Garnett). Only matters if volume gractions are given. Then, default is Sihvola.")
			;

		po::positional_options_description p;
		p.add("frequency",1);
		p.add("temperature",2);

		po::variables_map vm;
		po::store(po::command_line_parser(argc,argv).
			options(desc).positional(p).run(),
			vm );
		po::notify (vm);

		string method, sTemps, sFreqs, sNus, ofile;
		set<double> temps, freqs, nus;
		map<PHASE,string> vsVols;
		map<PHASE,set<double> > vVols;
		bool fileWrite = false;

		if (vm.count("output"))
		{
			fileWrite = true;
			ofile = vm["output"].as<string>();
		}
		if (vm.count("method"))
			method = vm["method"].as<string>();
		if (vm.count("temperature"))
			sTemps = vm["temperature"].as<string>();
		if (vm.count("frequency"))
			sFreqs = vm["frequency"].as<string>();
		if (vm.count("nu"))
			sNus = vm["nu"].as<string>();
		if (vm.count("volume-fraction-a"))
			vsVols[AIR] = vm["volume-fraction-a"].as<string>();
		if (vm.count("volume-fraction-wa"))
			vsVols[WATER] = vm["volume-fraction-w"].as<string>();
		if (vm.count("volume-fraction-i"))
			vsVols[ICE] = vm["volume-fraction-i"].as<string>();

		if (vm.count("help") || vm.size() == 0 || argc == 1 ||
			!vm.count("temperature") || !vm.count("frequency"))
		{
			cerr << desc << endl;
			exit(1);
		}

		rtmath::config::splitSet<double>(sTemps,temps);
		rtmath::config::splitSet<double>(sFreqs,freqs);
		rtmath::config::splitSet<double>(sNus,nus);
		for (auto it = vsVols.begin(); it != vsVols.end(); ++it)
		{
			set<double> s;
			rtmath::config::splitSet<double>(it->second,s);
			vVols[it->first] = move(s);
		}

		if (vm.count("mtab")) mtab = true;
		{
			using namespace boost::filesystem;
			path p(argv[0]);
			path fname = p.filename();
			if (fname == path("rtmath-genmtab"))
				mtab = true;
		}

		if (mtab)
		{
			if (temps.size() > 1 || freqs.size() > 1 || nus.size() > 1 || vVols.size() > 1)
			{
				throw rtmath::debug::xBadInput("mtab output requires only one frequency and temperature");
			}
		}

		bool mixed = false; // Mixed phase calculation
		if (vVols.size()) mixed = true;

		// Preprocessing done!
		
		// Figure out tuples of volume fractions
		// Tuple ordering is ice, water, air
		vector<boost::tuple<double, double, double> > fracs;
		{
			bool hasIce = false, hasWater = false, hasAir = false;
			if (vVols.count(ICE)) hasIce = true;
			if (vVols.count(WATER)) hasWater = true;
			if (vVols.count(AIR)) hasAir = true;
		
			// Set up the tuples by varying over possible known parameters and
			// attempting to fill in the unknowns.

			if (!hasIce && !hasWater && !hasAir)
			{
				// Do air / water determination at a later step.
				// This null tuple is detected later in the code
				fracs.push_back(boost::tuple<double,double,double>(0.0,0.0,0.0));
			} else {
				// This is ugly, but is shorter than coding graph theory
				if (!hasWater)
				{
					if (hasIce && hasAir)
					{
						for (auto ice = vVols[ICE].begin(); ice != vVols[ICE].end(); ++ice)
						{
							for (auto air = vVols[AIR].begin(); air != vVols[AIR].end(); ++air)
							{
								double water = 1.0 - *ice - *air;
								fracs.push_back( boost::tuple<double, double, double>(*ice, water, *air));
							}
						}
					} else {
						double water = 0;
						// Missing water and either ice or air
						if (hasAir)
						{
							for (auto air = vVols[AIR].begin(); air != vVols[AIR].end(); ++air)
							{
								double ice = 1.0 - *air;
								fracs.push_back(boost::tuple<double,double,double>(ice,water,*air));
							}
						}

						if (hasIce)
						{
							for (auto ice = vVols[ICE].begin(); ice != vVols[ICE].end(); ++ice)
							{
								double air = 1.0 - *ice;
								fracs.push_back(boost::tuple<double,double,double>(*ice,water,air));
							}
						}
					}
				} else {
					// has Water
					for (auto water = vVols[WATER].begin(); water != vVols[WATER].end(); ++water)
					{
						// Does it have ice, air, both or none?
						if (!hasIce & !hasAir)
						{
							double ice = 0;
							double air = 1.0 - *water;
							fracs.push_back(boost::tuple<double,double,double>(ice,*water,air));
						} else if (hasIce)
						{
							for (auto ice = vVols[ICE].begin(); ice != vVols[ICE].end(); ++ice)
							{
								double air = 1.0 - *water - *ice;
								fracs.push_back(boost::tuple<double,double,double>(*ice,*water,air));
							}
						} else {
							for (auto air = vVols[AIR].begin(); air != vVols[AIR].end(); ++air)
							{
								double ice = 1.0 - *water - *air;
								fracs.push_back(boost::tuple<double,double,double>(ice,*water,*air));
							}
						}
					}
				}
			}
		}

		// Actual calculations
		for (auto T = temps.begin(); T != temps.end(); ++T)
		{
			for (auto freq = freqs.begin(); freq != freqs.end(); ++freq)
			{
				for (auto nu = nus.begin(); nu != nus.end(); ++nu)
				{
					for (auto frac = fracs.begin(); frac != fracs.end(); ++frac)
					{
						// Call appropriate function depending on method
						complex<double> mAir(1.0,0);
						complex<double> mWat(1.0,0); // TODO: replace with actual f, T-dep value
						complex<double> mIce;
						rtmath::refract::mice(*freq,*T,mIce);

						double fIce = frac->get<0>();
						double fWat = frac->get<1>();
						double fAir = frac->get<2>();
						if (fIce == 0 && fWat == 0 && fAir == 0)
						{
							if (*T < 273.15)
							{
								fIce = 1.0;
							} else {
								fWat = 1.0;
							}
						}

						complex<double> mEff;

						if (method == "Sihvola")
						{
							rtmath::refract::sihvola(mIce,mAir,fIce,*nu,mEff);
						} else if (method == "Debye")
						{
							rtmath::refract::debyeDry(mIce,mAir,fIce, mEff);
						} else if (method == "Maxwell-Garnett")
						{
							if (fWat)
								throw rtmath::debug::xUnimplementedFunction(); // TODO: implement water refractive index
							rtmath::refract::maxwellGarnett(mIce,mWat,mAir,fIce,fWat,mEff);
						} else {
							throw rtmath::debug::xBadInput(method.c_str());
						}

						// Write effective refractive index
						if (!fileWrite)
						{
							write(mEff,mtab);
						} else {
							// Prep for special ROOT file writing
							throw rtmath::debug::xUnimplementedFunction();
						}
					}
				}
			}
		}

		if (fileWrite)
		{
			// Do special ROOT file writing
			throw rtmath::debug::xUnimplementedFunction();
		}

	}
	catch (std::exception &e)
	{
		cerr << "Exception caught\n";
		cerr << e.what() << endl;
		return 1;
	}

	return 0;
}

void write(const std::complex<double> &ref, bool mtab)
{
	using namespace std;
//	rtmath::refract::mice(f,temp,ref);
	cout.setf( ios::scientific, ios::floatfield);
	cout.precision(7);
	if (!mtab)
	{
		cout << " ( " << ref.real() << " , " << ref.imag() << " ) " << endl;
	} else {
		cout << " m = " << ref.real() << " + " << (-1.0 *ref.imag()) << " i" << endl;
		cout << " 1 2 3 0 0 = columns for wave, Re(n), Im(n), eps1, eps2" << endl;
		cout << " LAMBDA  Re(N)   Im(N)" << endl;
		cout << " 0.000001    " << ref.real() << "      " << (-1.0*ref.imag()) << endl;
		cout << " 1.000000    " << ref.real() << "      " << (-1.0*ref.imag()) << endl;
		cout << " 100000.0    " << ref.real() << "      " << (-1.0*ref.imag()) << endl;
	}

}
