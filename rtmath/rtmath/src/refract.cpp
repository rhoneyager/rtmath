// Code segment directly based on Liu's code. Rewritten in C++ so that it may be 
// compiled in an MSVC environment

#include "Stdafx-core.h"
#include <cmath>
#include <complex>
#include <fstream>
#include "../rtmath/refract.h"
#include "../rtmath/zeros.h"
#include "../rtmath/error/error.h"

namespace {
	boost::program_options::options_description SHARED_PRIVATE *pcmdline = nullptr;
	boost::program_options::options_description SHARED_PRIVATE *pconfig = nullptr;
	boost::program_options::options_description SHARED_PRIVATE *phidden = nullptr;
	double nu = 0;
	std::complex<double> m_force;
}

void rtmath::refract::add_options(
	boost::program_options::options_description &cmdline,
	boost::program_options::options_description &config,
	boost::program_options::options_description &hidden)
{
	namespace po = boost::program_options;
	using std::string;

	pcmdline = &cmdline;
	pconfig = &config;
	phidden = &hidden;

	/// \todo Add option for default rtmath.conf location

	cmdline.add_options()
		("nu,n", po::value<double>()->default_value(0.85), "Value of nu for Sihvola refractive index scaling")
		("mr", po::value<double>(), "Override real refractive index value")
		("mi", po::value<double>(), "Override imaginary refractive index value")
		("refractive-method", po::value<string>()->default_value("mg-ellipsoids"),
		"Specify the dielectric method to use in calculations (mg-spheres, "
		"mg-ellipsoids, sihvola, bruggeman, debye, maxwell-garnett (mg) )")
		;

	config.add_options()
		;

	hidden.add_options()
		;
}

void rtmath::refract::process_static_options(
	boost::program_options::variables_map &vm)
{
	namespace po = boost::program_options;
	using std::string;

	if (vm.count("nu")) nu = vm["nu"].as<double>();
	if (vm.count("mr") || vm.count("mi"))
	{
		m_force = std::complex<double>(vm["mr"].as<double>(),
			vm["mi"].as<double>());
	}

	// Add in refractive method calculations here.
}

void rtmath::refract::mWater(double f, double t, std::complex<double> &m)
{
	if (f< 0) RTthrow rtmath::debug::xModelOutOfRange(f);
	if (f < 1000)
	{
		if (t >= 273)
		{
			mWaterLiebe(f,t,m);
		} else {
			throw rtmath::debug::xModelOutOfRange(t);
		}
	} else {
		throw rtmath::debug::xModelOutOfRange(f);
	}
}

void rtmath::refract::mIce(double f, double t, std::complex<double> &m)
{
	if (f< 0) RTthrow rtmath::debug::xModelOutOfRange(f);
	if (f < 1000)
	{
		if (t <= 278)
		{
			mIceMatzler(f,t,m);
		} else {
			throw rtmath::debug::xModelOutOfRange(t);
		}
	} else {
		throw rtmath::debug::xModelOutOfRange(f);
	}
}

// Water complex refractive index
// from Liu's mcx.f
// LIEBE, HUFFORD AND MANABE, INT. J. IR & MM WAVES V.12, pp.659-675
//  (1991);  Liebe et al, AGARD Conf. Proc. 542, May 1993.
// Valid from 0 to 1000 GHz. freq in GHz, temp in K
void rtmath::refract::mWaterLiebe(double f, double t, std::complex<double> &m)
{
	if (f < 0 || f > 1000)
		throw rtmath::debug::xModelOutOfRange(f);
	double theta1 = 1.0 - (300.0/t);
	double eps0 = 77.66 - (103.3*theta1);
	double eps1 = .0671*eps0;
	double eps2 = 3.52;
	double fp = (316.*theta1 + 146.4)*theta1 + 20.20;
	double fs = 39.8*fp;
	using namespace std;
	complex<double> eps;
	eps = complex<double>(eps0-eps1,0)/complex<double>(1.0,f/fp)
		+ complex<double>(eps1-eps2,0)/complex<double>(1.0,f/fs)
		+ complex<double>(eps2,0);
	m = sqrt(eps);
}

// Ice complex refractive index
// based on Christian Matzler (2006)
void rtmath::refract::mIceMatzler(double f, double t, std::complex<double> &m)
{
	double er = 0;
	if (t>243.0)
		er = 3.1884+9.1e-4*(t-273.0);
	else
		er = 3.1611+4.3e-4*(t-243.0);
	// Imaginary part
	double theta = 300.0/(t)-1.0;
	double alpha = (0.00504+0.0062*theta)*exp(-22.1*theta);
	double dbeta = exp(-9.963+0.0372*(t-273.16));
	const double B1 = 0.0207;
	const double B2 = 1.16e-11;
	const double b = 335;
	double betam = B1/t*exp(b/t)/pow((exp(b/t)-1.0),2)+B2*pow(f,2.0);
	double beta = betam+dbeta;
	double ei = alpha/f+beta*f;
	std::complex<double> e(er,-ei);
	m = sqrt(e);
}

void rtmath::refract::writeDiel(const std::string &filename, 
								const std::complex<double> &ref)
{
	using namespace std;
	ofstream out(filename.c_str());
	out.setf( ios::scientific, ios::floatfield);
	out.precision(7);
	out << " m = " << ref.real() << " + " << (-1.0 *ref.imag()) << " i" << endl;
	out << " 1 2 3 0 0 = columns for wave, Re(n), Im(n), eps1, eps2" << endl;
	out << " LAMBDA  Re(N)   Im(N)" << endl;
	out << " 0.000001    " << ref.real() << "      " << (-1.0*ref.imag()) << endl;
	out << " 1.000000    " << ref.real() << "      " << (-1.0*ref.imag()) << endl;
	out << " 100000.0    " << ref.real() << "      " << (-1.0*ref.imag()) << endl;
}


void rtmath::refract::bruggeman(std::complex<double> Ma, std::complex<double> Mb, 
								double fa, std::complex<double> &Mres)
{
	using namespace std;
	complex<double> eA, eB, eRes;
	mToE(Ma,eA);
	mToE(Mb,eB);

	// Liu's formula is derived only for mAmbient = 1. I want a more general case to make 
	// dielectric chaining easier.
	auto formula = [&](std::complex<double> x) -> std::complex<double>
	{
		// f (( eA - eE ) / (eA + 2eE) ) + (1-f) (( eB - eE ) / (eB + 2eE) ) = 0
		using namespace std;
		complex<double> res, pa, pb;
		pa = complex<double>(fa,0) * (eA - x) / (eA + (complex<double>(2.,0)*x));
		pb = complex<double>(1.-fa,0) * (eB - x) / (eB + (complex<double>(2.,0)*x));
		res = pa + pb;
		return res;
	};

	complex<double> gA(1.0,1.0), gB(1.55,1.45);
	eRes = zeros::secantMethod(formula, gA, gB);

	eToM(eRes,Mres);
}

void rtmath::refract::debyeDry(std::complex<double> Ma, std::complex<double> Mb, 
							   double fa, std::complex<double> &Mres)
{
	using namespace std;
	std::complex<double> eA, eB, eRes;
	mToE(Ma,eA);
	mToE(Mb,eB);

	complex<double> pa = fa * (eA - complex<double>(1.,0)) / (eA + complex<double>(2.,0));
	complex<double> pb = (1.-fa) * (eB - complex<double>(1.,0)) / (eB + complex<double>(2.,0));

	complex<double> fact = pa+pb;
	eRes = (complex<double>(2.,0) * fact + complex<double>(1.,0)) 
		/ (complex<double>(1.,0) - fact);
	eToM(eRes,Mres);
}

void rtmath::refract::maxwellGarnettSpheres(std::complex<double> Ma, std::complex<double> Mb, 
											double fa, std::complex<double> &Mres)
{
	using namespace std;
	std::complex<double> eA, eB, eRes;
	mToE(Ma,eA);
	mToE(Mb,eB);

	// Formula is:
	// (e_eff - eb)/(e_eff+2eb) = fa * (ea - eb) / (ea + 2 eb)
	complex<double> a = complex<double>(fa,0) * ( eA - eB ) / (eA + (complex<double>(2,0) * eB));
	eRes = eB * (complex<double>(2,0) * a + complex<double>(1,0))
		/ (complex<double>(1,0) - a);
	eToM(eRes,Mres);
}

void rtmath::refract::maxwellGarnettEllipsoids(std::complex<double> Ma, std::complex<double> Mb, 
											   double fa, std::complex<double> &Mres)
{
	using namespace std;
	std::complex<double> eA, eB, eRes;
	mToE(Ma,eA);
	mToE(Mb,eB);

	complex<double> betaA = complex<double>(2.,0)*eB/(eA-eB);
	complex<double> betaB = ( ((eA)/(eA-eB)) * log(eA/eB) ) - complex<double>(1.,0);
	complex<double> beta = betaA * betaB;

	complex<double> cf(fa,0), cfc(1.-fa,0);

	eRes = ( (cfc*eB) + (cf*beta*eA) )/(cfc + (cf*beta) );
	eToM(eRes,Mres);
}

void rtmath::refract::sihvola(std::complex<double> Ma, std::complex<double> Mb, 
							  double fa, double nu, std::complex<double> &Mres)
{
	using namespace std;
	std::complex<double> eA, eB, eRes;
	mToE(Ma,eA);
	mToE(Mb,eB);

	/** Formula is:
	* (e_eff - eb) / (e_eff + 2eb + v(e_eff - eb)) - fa (ea-eb)/(ea+2eb+v(e_eff-eb) = 0
	* This formula has no analytic solution. It is also complex-valued. Its derivative is hard to calculate.
	* Because of this, I will be using the complex secant method to find the zeros.
	* This is also why the other mixing formulas do not call this code.
	**/

	auto formulaSihvola = [&](std::complex<double> x) -> std::complex<double>
	{
		using namespace std;
		complex<double> res, pa, pb;
		pa = (x-eB)/(x+(complex<double>(2.0,0)*eB)+(complex<double>(nu,0)*(x-eB)));
		pb = complex<double>(fa,0) * 
			(eA-eB)/(eA+(complex<double>(2.0,0)*eB)+(complex<double>(nu,0)*(x-eB)));
		res = pa - pb;
		return res;
	};

	complex<double> gA(1.0,1.0), gB(1.55,1.45);
	eRes = zeros::secantMethod(formulaSihvola, gA, gB);
	eToM(eRes,Mres);
}

void rtmath::refract::mToE(std::complex<double> m, std::complex<double> &e)
{
	e = m * m;
}

void rtmath::refract::eToM(std::complex<double> e, std::complex<double> &m)
{
	m = sqrt(e);
}


void rtmath::refract::MultiInclusions(
	const std::vector<basicDielectricTransform> &funcs,
	const std::vector<double> &fs, 
	const std::vector<std::complex<double> > &ms, 
	std::complex<double> &Mres)
{
	//if (fs.size() != ms.size() + 1) RTthrow rtmath::debug::xBadInput("Array sizes are not the same");
	//if (fs.size() != funcs.size()) RTthrow rtmath::debug::xBadInput("Array sizes are not the same");

	using namespace std;

	double fTot = 0.0;
	double fEnv = 1.0;

	Mres = ms.at(0);
	fTot = fs.at(0);

	// Ordering is most to least enclosed
	for (size_t i=1; i<funcs.size(); ++i)
	{
		// ms[0] is the initial refractive index (innermost material), and fs[0] is its volume fraction
		complex<double> mA = Mres;
		fTot += fs.at(i);
		if (!fTot) continue;
		funcs.at(i)(mA, ms.at(i), fs.at(i-1) / fTot, Mres);
	}
}


/*
void rtmath::refract::maxwellGarnett(std::complex<double> Mice, std::complex<double> Mwater, 
std::complex<double> Mair, double fIce, double fWater, std::complex<double> &Mres)
{
using namespace std;
std::complex<double> Miw;

// Ice is the inclusion in water, which is the inclusion in air
double frac = 0;
if (fWater+fIce == 0) frac = 0;
else frac = fIce / (fWater + fIce);
maxwellGarnettSpheres(Mice, Mwater, frac, Miw);
maxwellGarnettSpheres(Miw, Mair, fIce + fWater, Mres);
}
*/
