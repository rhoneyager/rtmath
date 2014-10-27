// Code segment directly based on Liu's code. Rewritten in C++ so that it may be 
// compiled in an MSVC environment

#include "Stdafx-core.h"
#include <cmath>
#include <complex>
#include <fstream>
#include <valarray>
#include "../rtmath/refract.h"
#include "../rtmath/zeros.h"
#include "../rtmath/units.h"
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
	throw;
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
		if (t <= 278)
		{
			mIceWarren(f, t, m);
		}
		else {
			throw rtmath::debug::xModelOutOfRange(t);
		}
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

void rtmath::refract::mIceWarren(double f, double t, std::complex<double> &m)
{
	// Warren table 2 is used for interpolation
	// Data written out to a structure
	const size_t numTs = 4;
	double tempCs[numTs] = { -1, -5, -20, -60 };
	double tempC = t - 273.15;
	if (tempC > 0 || tempC < tempCs[3]) throw rtmath::debug::xModelOutOfRange(t);
	// Column format is wavelength (mm), m_re annd m_im for each temperature
	// size is 62*9 = 558 elements
	const size_t cols = (numTs * 2) + 1, rows = 62;
	size_t numElems = rows*cols;
	size_t maxwvlen = numElems - cols;
	// First wavelength is at tbl[0], last is at tbl[549]
	double tbl[] = {
		.1670, 1.8296, 8.30e-2, 1.8296, 8.30e-2, 1.8296, 8.30e-2, 1.8296, 8.30e-2,
		.1778, 1.8236, 6.90e-2, 1.8236, 6.90e-2, 1.8236, 6.90e-2, 1.8236, 6.90e-2,
		.1884, 1.8315, 5.70e-2, 1.8315, 5.70e-2, 1.8315, 5.70e-2, 1.8315, 5.70e-2,
		.1995, 1.8275, 4.56e-2, 1.8275, 4.56e-2, 1.8275, 4.56e-2, 1.8275, 4.45e-2,
		.2113, 1.8222, 3.79e-2, 1.8222, 3.79e-2, 1.8222, 3.79e-2, 1.8222, 3.55e-2,
		.2239, 1.8172, 3.14e-2, 1.8172, 3.14e-2, 1.8172, 3.14e-2, 1.8172, 2.91e-2,
		.2371, 1.8120, 2.62e-2, 1.8120, 2.62e-2, 1.8120, 2.62e-2, 1.8120, 2.44e-2,
		.2512, 1.8070, 2.24e-2, 1.8070, 2.24e-2, 1.8070, 2.19e-2, 1.8070, 1.97e-2,
		.2661, 1.8025, 1.96e-2, 1.8025, 1.96e-2, 1.8025, 1.88e-2, 1.8025, 1.67e-2,
		.2818, 1.7983, 1.76e-2, 1.7983, 1.76e-2, 1.7983, 1.66e-2, 1.7983, 1.40e-2,
		.2985, 1.7948, 1.67e-2, 1.7948, 1.67e-2, 1.7948, 1.54e-2, 1.7948, 1.26e-2,
		.3162, 1.7921, 1.62e-2, 1.7921, 1.60e-2, 1.7921, 1.47e-2, 1.7921, 1.08e-2,
		.3548, 1.7884, 1.55e-2, 1.7884, 1.50e-2, 1.7884, 1.35e-2, 1.7884, 8.90e-3,
		.3981, 1.7860, 1.47e-2, 1.7860, 1.40e-2, 1.7860, 1.25e-2, 1.7860, 7.34e-3,
		.4467, 1.7843, 1.39e-2, 1.7843, 1.31e-2, 1.7843, 1.15e-2, 1.7843, 6.40e-3,
		.5012, 1.7832, 1.32e-2, 1.7832, 1.23e-2, 1.7832, 1.06e-2, 1.7832, 5.60e-3,
		.5623, 1.7825, 1.25e-2, 1.7825, 1.15e-2, 1.7825, 9.77e-3, 1.7825, 5.00e-3,
		.6310, 1.7820, 1.18e-2, 1.7820, 1.08e-2, 1.7820, 9.01e-3, 1.7820, 4.52e-3,
		.7943, 1.7817, 1.06e-2, 1.7817, 9.46e-3, 1.7816, 7.66e-3, 1.7815, 3.68e-3,
		1.000, 1.7816, 9.54e-3, 1.7816, 8.29e-3, 1.7814, 6.52e-3, 1.7807, 2.99e-3,
		1.259, 1.7819, 8.56e-3, 1.7819, 7.27e-3, 1.7816, 5.54e-3, 1.7801, 2.49e-3,
		2.500, 1.7830, 6.21e-3, 1.7830, 4.91e-3, 1.7822, 3.42e-3, 1.7789, 1.55e-3,
		5.000, 1.7843, 4.49e-3, 1.7843, 3.30e-3, 1.7831, 2.10e-3, 1.7779, 9.61e-4,
		10.00, 1.7852, 3.24e-3, 1.7852, 2.22e-3, 1.7838, 1.29e-3, 1.7773, 5.95e-4,
		20.00, 1.7862, 2.34e-3, 1.7861, 1.49e-3, 1.7839, 7.93e-4, 1.7772, 3.69e-4,
		32.00, 1.7866, 1.88e-3, 1.7863, 1.14e-3, 1.7840, 5.70e-4, 1.7772, 2.67e-4,
		35.00, 1.7868, 1.74e-3, 1.7864, 1.06e-3, 1.7840, 5.35e-4, 1.7772, 2.51e-4,
		40.00, 1.7869, 1.50e-3, 1.7865, 9.48e-4, 1.7840, 4.82e-4, 1.7772, 2.29e-4,
		45.00, 1.7870, 1.32e-3, 1.7865, 8.50e-4, 1.7840, 4.38e-4, 1.7772, 2.11e-4,
		50.00, 1.7870, 1.16e-3, 1.7865, 7.66e-4, 1.7840, 4.08e-4, 1.7772, 1.96e-4,
		60.00, 1.7871, 8.80e-4, 1.7865, 6.30e-4, 1.7839, 3.50e-4, 1.7772, 1.73e-4,
		70.00, 1.7871, 6.95e-4, 1.7865, 5.20e-4, 1.7838, 3.20e-4, 1.7772, 1.55e-4,
		90.00, 1.7872, 4.64e-4, 1.7865, 3.84e-4, 1.7837, 2.55e-4, 1.7772, 1.31e-4,
		111.0, 1.7872, 3.40e-4, 1.7865, 2.96e-4, 1.7837, 2.12e-4, 1.7772, 1.13e-4,
		120.0, 1.7872, 3.11e-4, 1.7865, 2.70e-4, 1.7837, 2.00e-4, 1.7772, 1.06e-4,
		130.0, 1.7872, 2.94e-4, 1.7865, 2.52e-4, 1.7837, 1.86e-4, 1.7772, 9.90e-5,
		140.0, 1.7872, 2.79e-4, 1.7865, 2.44e-4, 1.7837, 1.75e-4, 1.7772, 9.30e-5,
		150.0, 1.7872, 2.70e-4, 1.7865, 2.36e-4, 1.7837, 1.66e-4, 1.7772, 8.73e-5,
		160.0, 1.7872, 2.64e-4, 1.7865, 2.30e-4, 1.7837, 1.56e-4, 1.7772, 8.30e-5,
		170.0, 1.7872, 2.58e-4, 1.7865, 2.28e-4, 1.7837, 1.49e-4, 1.7772, 7.87e-5,
		180.0, 1.7872, 2.52e-4, 1.7865, 2.25e-4, 1.7837, 1.44e-4, 1.7772, 7.50e-5,
		200.0, 1.7872, 2.49e-4, 1.7865, 2.20e-4, 1.7837, 1.35e-4, 1.7772, 6.83e-5,
		250.0, 1.7872, 2.54e-4, 1.7865, 2.16e-4, 1.7837, 1.21e-4, 1.7772, 5.60e-5,
		290.0, 1.7872, 2.64e-4, 1.7865, 2.17e-4, 1.7837, 1.16e-4, 1.7772, 4.96e-5,
		320.0, 1.7872, 2.74e-4, 1.7865, 2.20e-4, 1.7837, 1.16e-4, 1.7772, 4.55e-5,
		350.0, 1.7872, 2.89e-4, 1.7665, 2.25e-4, 1.7837, 1.17e-4, 1.7772, 4.21e-5,
		380.0, 1.7872, 3.05e-4, 1.7865, 2.32e-4, 1.7837, 1.20e-4, 1.7772, 3.91e-5,
		400.0, 1.7872, 3.15e-4, 1.7865, 2.39e-4, 1.7837, 1.23e-4, 1.7772, 3.76e-5,
		450.0, 1.7872, 3.46e-4, 1.7865, 2.60e-4, 1.7837, 1.32e-4, 1.7772, 3.40e-5,
		500.0, 1.7872, 3.82e-4, 1.7865, 2.86e-4, 1.7837, 1.44e-4, 1.7772, 3.10e-5,
		600.0, 1.7872, 4.62e-4, 1.7865, 3.56e-4, 1.7837, 1.68e-4, 1.7772, 2.64e-5,
		640.0, 1.7872, 5.00e-4, 1.7865, 3.83e-4, 1.7837, 1.80e-4, 1.7772, 2.51e-5,
		680.0, 1.7872, 5.50e-4, 1.7865, 4.15e-4, 1.7837, 1.90e-4, 1.7772, 2.43e-5,
		720.0, 1.7872, 5.95e-4, 1.7865, 4.45e-4, 1.7837, 2.09e-4, 1.7772, 2.39e-5,
		760.0, 1.7872, 6.47e-4, 1.7865, 4.76e-4, 1.7837, 2.16e-4, 1.7772, 2.37e-5,
		800.0, 1.7872, 6.92e-4, 1.7865, 5.08e-4, 1.7837, 2.29e-4, 1.7772, 2.38e-5,
		840.0, 1.7872, 7.42e-4, 1.7865, 5.40e-4, 1.7837, 2.40e-4, 1.7772, 2.40e-5,
		900.0, 1.7872, 8.20e-4, 1.7865, 5.86e-4, 1.7837, 2.60e-4, 1.7772, 2.46e-5,
		1000., 1.7872, 9.70e-4, 1.7865, 6.78e-4, 1.7837, 2.92e-4, 1.7772, 2.66e-5,
		2000., 1.7872, 1.95e-3, 1.7865, 1.28e-3, 1.7837, 6.10e-4, 1.7772, 4.45e-5,
		5000., 1.7872, 5.78e-3, 1.7865, 3.55e-3, 1.7840, 1.02e-3, 1.7772, 8.70e-5,
		8600., 1.7880, 9.70e-3, 1.7872, 5.60e-3, 1.7845, 1.81e-3, 1.7780, 1.32e-4
	};
	// first, interpolate according to frequency, then by temperature

	double wvlen = rtmath::units::conv_spec("GHz", "mm").convert(f);
	if (wvlen < tbl[0] || wvlen > tbl[maxwvlen]) throw rtmath::debug::xModelOutOfRange(f);


	std::valarray<double> tblV(tbl, numElems);
	std::valarray<double> wvlens = tblV[std::slice(0, rows, cols)];
	//std::vector<float> v_wvlens(std::begin(wvlens), std::end(wvlens));
	size_t lowFreqIndex = 0;
	auto it = std::find_if(std::begin(wvlens), std::end(wvlens), [&](float w) -> bool {
		if (w > wvlen) return true;
		++lowFreqIndex;
		return false; });
	size_t highFreqIndex = lowFreqIndex+1;
	if (it == std::end(wvlens)) throw rtmath::debug::xModelOutOfRange(f); // should not happen
	auto ot = ++it;
	if (ot == std::end(wvlens)) throw rtmath::debug::xModelOutOfRange(f); // should not happen
	// Interpolate temperature entries based on frequency interpolation
	double ffrac = (wvlen - *it) / (*ot - *it);
	double tints[cols];
	tints[0] = wvlen;
	for (size_t i = 1; i < cols; ++i)
		tints[i] = (ffrac*tbl[(cols*lowFreqIndex) + i]) + ((1 - ffrac)*tbl[(cols*highFreqIndex) + i]);
	
	// Interpolate temperature elements
	size_t lowTempIndex = numTs - 1;
	while (lowTempIndex && tempCs[lowTempIndex] > tempC) { lowTempIndex--; }
	double tfrac = (tempC - tempCs[lowTempIndex]) / (tempCs[lowTempIndex + 1] - tempCs[lowTempIndex]);
	std::complex<double> mRes(
		(tfrac*tints[1 + (2 * lowTempIndex)]) + ((1 - tfrac)*tints[1 + (2 * (lowTempIndex-1))]),
		(tfrac*tints[2 + (2 * lowTempIndex)]) + ((1 - tfrac)*tints[2 + (2 * (lowTempIndex - 1))]));
	m = mRes;
	//std::find_if(tbl, tbl + maxwvlen + 1, []());
}

void rtmath::refract::mWaterHanel(double lambda, std::complex<double> &m)
{
	const size_t nWvlengths = 90;
	double wavelengths[nWvlengths] = { 0.2, 0.25, 0.3, 0.337, 0.4, 0.488, 0.515,
		0.55, 0.633, 0.694, 0.86, 1.06, 1.3, 1.536, 1.8, 2, 2.25, 2.5,
		2.7, 3, 3.2, 3.392, 3.5, 3.75, 4, 4.5, 5, 5.5, 6, 6.2, 6.5,
		7.2, 7.9, 8.2, 8.5, 8.7, 9, 9.2, 9.5, 9.8, 10, 10.591, 11,
		11.5, 12.5, 13, 14, 14.8, 15, 16.4, 17.2, 18, 18.5, 20, 21.3,
		22.5, 25, 27.9, 30, 35, 40, 45, 50, 55, 60, 65, 70, 80, 90,
		100, 110, 120, 135, 150, 165, 180, 200, 250, 300, 400, 500,
		750, 1000, 1500, 2000, 3000, 5000, 10000, 20000, 30000 };
	double water_re[nWvlengths] = { 1.396, 1.362, 1.349, 1.345, 1.339, 1.335, 
		1.334, 1.333, 1.332, 1.331, 1.329, 1.326, 1.323, 1.318, 1.312, 1.306, 
		1.292, 1.261, 1.188, 1.371, 1.478, 1.422, 1.4, 1.369, 1.351, 1.332, 
		1.325, 1.298, 1.265, 1.363, 1.339, 1.312, 1.294, 1.286, 1.278, 1.272, 
		1.262, 1.255, 1.243, 1.229, 1.218, 1.179, 1.153, 1.126, 1.123, 1.146, 
		1.21, 1.258, 1.27, 1.346, 1.386, 1.423, 1.443, 1.48, 1.491, 1.506, 1.531, 
		1.549, 1.551, 1.532, 1.519, 1.536, 1.587, 1.645, 1.703, 1.762, 1.821, 
		1.92, 1.979, 2.037, 2.06, 2.082, 2.094, 2.106, 2.109, 2.113, 2.117, 
		2.12, 2.121, 2.142, 2.177, 2.291, 2.437, 2.562, 2.705, 3.013, 3.627, 
		4.954, 6.728, 7.682 };
	double water_im[nWvlengths] = { 1.10E-07, 3.35E-08, 1.60E-08, 8.45E-09, 
		1.86E-09, 9.69E-10, 1.18E-09, 1.96E-09, 1.46E-08, 3.05E-08, 3.29E-07, 
		4.18E-06, 3.69E-05, 9.97E-05, 1.15E-04, 1.10E-03, 3.90E-04, 0.00174, 
		0.019, 0.272, 0.0924, 0.0204, 0.0094, 0.0035, 0.0046, 0.0134, 0.0124, 
		0.0116, 0.107, 0.088, 0.0392, 0.0321, 0.0339, 0.0351, 0.0367, 0.0379, 
		0.0399, 0.0415, 0.0444, 0.0479, 0.0508, 0.0674, 0.0968, 0.142, 0.259, 
		0.305, 0.37, 0.396, 0.402, 0.427, 0.429, 0.426, 0.421, 0.393, 0.379, 
		0.37, 0.356, 0.339, 0.328, 0.336, 0.385, 0.449, 0.514, 0.551, 0.587, 
		0.582, 0.576, 0.52, 0.49, 0.46, 0.44, 0.42, 0.41, 0.4, 0.41, 0.42, 
		0.43, 0.46, 0.49, 0.53, 0.58, 0.65, 0.73, 0.96, 1.19, 1.59, 2.14, 2.79, 
		2.87, 2.51 };
	double ice_re[nWvlengths] = { 1.394, 1.351, 1.334, 1.326, 1.32, 1.313, 1.312, 
		1.311, 1.308, 1.306, 1.303, 1.3, 1.295, 1.29, 1.282, 1.273, 1.256, 1.225, 
		1.163, 1.045, 1.652, 1.51, 1.453, 1.391, 1.361, 1.34, 1.327, 1.299, 1.296, 
		1.313, 1.32, 1.318, 1.313, 1.306, 1.291, 1.282, 1.269, 1.261, 1.245, 1.219, 
		1.197, 1.098, 1.093, 1.176, 1.387, 1.472, 1.569, 1.579, 1.572, 1.531, 1.534, 
		1.522, 1.51, 1.504, 1.481, 1.455, 1.414, 1.358, 1.325, 1.226, 1.202, 1.299, 
		1.629, 1.767, 1.585, 1.748, 1.869, 1.903, 1.856, 1.832, 1.821, 1.819, 1.819, 
		1.823, 1.829, 1.832, 1.827, 1.807, 1.795, 1.786, 1.783, 1.782, 1.781, 1.782, 
		1.782, 1.782, 1.783, 1.784, 1.784, 1.784 };
	double ice_im[nWvlengths] = { 1.50E-08, 8.60E-09, 5.50E-09, 4.50E-09, 2.71E-09, 
		1.75E-09, 2.19E-09, 3.11E-09, 1.09E-08, 2.62E-08, 2.15E-07, 1.96E-06, 
		1.32E-05, 6.10E-04, 1.13E-04, 1.61E-03, 2.13E-04, 7.95E-04, 0.00293, 0.429, 
		0.283, 0.0401, 0.0161, 0.007, 0.01, 0.0287, 0.012, 0.0217, 0.0647, 0.0683, 
		0.0559, 0.0544, 0.0479, 0.039, 0.0391, 0.04, 0.0429, 0.0446, 0.0459, 0.047, 
		0.051, 0.131, 0.239, 0.36, 0.422, 0.389, 0.283, 0.191, 0.177, 0.125, 0.107, 
		0.0839, 0.076, 0.067, 0.0385, 0.0291, 0.0299, 0.049, 0.065, 0.155, 0.344, 
		0.601, 0.543, 0.42, 0.39, 0.49, 0.399, 0.235, 0.165, 0.139, 0.126, 0.12, 
		0.108, 0.0962, 0.0846, 0.065, 0.0452, 0.0222, 0.0153, 0.0125, 0.0106, 0.008, 
		0.0065, 0.0049, 0.004, 0.003, 0.0021, 0.0013, 7.90E-04, 5.90E-04 };
	double nacl_re[nWvlengths] = { 1.79, 1.655, 1.607, 1.587, 1.567, 1.553, 1.55, 
		1.547, 1.542, 1.539, 1.534, 1.531, 1.529, 1.528, 1.527, 1.527, 1.526, 1.525, 
		1.525, 1.524, 1.524, 1.523, 1.523, 1.522, 1.522, 1.52, 1.519, 1.517, 1.515, 
		1.515, 1.513, 1.51, 1.507, 1.505, 1.504, 1.503, 1.501, 1.5, 1.498, 1.496, 
		1.495, 1.491, 1.488, 1.484, 1.476, 1.471, 1.462, 1.454, 1.451, 1.435, 1.425, 
		1.414, 1.406, 1.382, 1.36, 1.33, 1.27, 1.17, 1.08, 0.78, 0.58, 0.27, 0.14, 
		0.31, 4.52, 5.28, 3.92, 3.17, 2.87, 2.74, 2.64, 2.59, 2.54, 2.5, 2.48, 2.47, 
		2.45, 2.44, 2.43, 2.43, 2.43, 2.43, 2.43, 2.43, 2.43, 2.43, 2.43, 2.43, 2.43, 2.43 };
	double nacl_im[nWvlengths] = { 3.10E-09, 2.30E-09, 1.50E-09, 8.70E-10, 3.80E-10, 
		1.10E-10, 4.90E-11, 6.80E-11, 1.10E-10, 1.50E-10, 2.40E-10, 3.50E-10, 
		4.80E-10, 6.10E-10, 7.50E-10, 8.60E-10, 1.00E-09, 1.10E-09, 1.20E-09, 
		1.40E-09, 1.50E-09, 1.60E-09, 1.65E-09, 1.80E-09, 1.80E-09, 1.80E-09, 
		1.70E-09, 2.60E-09, 4.90E-09, 5.80E-09, 7.20E-09, 1.00E-08, 1.40E-08, 
		1.50E-08, 1.60E-08, 1.70E-08, 1.90E-08, 2.00E-08, 3.00E-08, 4.40E-08, 
		5.30E-08, 8.00E-08, 1.30E-07, 3.30E-07, 1.40E-06, 2.80E-06, 8.80E-06, 
		2.30E-05, 2.70E-05, 7.60E-05, 1.30E-04, 2.00E-04, 2.90E-04, 6.20E-04, 
		9.90E-04, 0.0014, 0.0035, 0.01, 0.026, 0.14, 0.66, 1.08, 1.99, 3.46, 6.94, 
		0.761, 0.271, 0.123, 0.0968, 0.087, 0.079, 0.077, 0.072, 0.064, 0.056, 0.052, 
		0.047, 0.041, 0.03, 0.027, 0.024, 0.012, 0.008, 0.0061, 0.0047, 0.0029, 
		0.0024, 5.60E-04, 4.10E-04, 2.60E-04 };
	double seasalt_re[nWvlengths] = { 1.51, 1.51, 1.51, 1.51, 1.5, 1.5, 1.5, 
		1.5, 1.49, 1.49, 1.48, 1.47, 1.47, 1.46, 1.45, 1.45, 1.44, 1.43, 1.4, 
		1.61, 1.49, 1.48, 1.48, 1.47, 1.48, 1.49, 1.47, 1.42, 1.41, 1.6, 1.46, 
		1.42, 1.4, 1.42, 1.48, 1.6, 1.65, 1.61, 1.58, 1.56, 1.54, 1.5, 1.48, 
		1.48, 1.42, 1.41, 1.41, 1.43, 1.45, 1.56, 1.74, 1.78, 1.77, 1.76, 1.76, 
		1.76, 1.76, 1.77, 1.77, 1.76, 1.74, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	double seasalt_im[nWvlengths] = { 1.00E-04, 5.00E-06, 2.00E-06, 4.00E-07, 
		3.00E-08, 2.00E-08, 1.00E-08, 1.00E-08, 2.00E-08, 1.00E-07, 3.00E-06, 
		2.00E-04, 4.00E-04, 6.00E-04, 8.00E-04, 0.001, 0.002, 0.004, 0.007, 
		0.01, 0.003, 0.002, 0.0016, 0.0014, 0.0014, 0.0014, 0.0025, 0.0036, 
		0.011, 0.022, 0.005, 0.007, 0.013, 0.02, 0.026, 0.03, 0.028, 0.026, 
		0.018, 0.016, 0.015, 0.014, 0.014, 0.014, 0.016, 0.018, 0.023, 0.03, 
		0.035, 0.09, 0.12, 0.13, 0.135, 0.152, 0.165, 0.18, 0.205, 0.275, 0.3, 
		0.5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0 };

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

	complex<double> betaA = complex<double>(2., 0)*eA / (eB - eA);
	complex<double> betaB = ((eB / (eB - eA)) * log(eB / eA)) - complex<double>(1., 0);
	complex<double> beta = betaA * betaB;

	complex<double> cf(fa,0), cfc(1.-fa,0);

	eRes = ((cfc*beta) + (cf*eA)) / (cf + (cfc*beta));
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

double rtmath::refract::guessTemp(double freq, const std::complex<double>& m,
	std::function<void(double freq, double temp, std::complex<double>& mres)> meth)
{
	using namespace std;
	try {
		// Attempt to guess the formula using the secant method.
		auto formulaTemp = [&](double T) -> double
		{
			// 0 = mRes(f,T) - m_known
			using namespace std;
			complex<double> mRes;
			//try {
				meth(freq, T, mRes);
			//}
			//catch (rtmath::debug::xModelOutOfRange &) {
			//	rtmath::refract::mWater(freq, T, mRes);
			//}

			double mNorm = mRes.real() - m.real(); //norm(mRes- ms.at(0));

			//std::cerr << "fT: " << T << "\t" << mRes << "\t\t" << ms.at(0) << "\t" << mNorm << std::endl;
			return mNorm;
		};

		double TA = 263, TB = 233, temp = 0;
		//complex<double> gA, gB;
		//refract::mIce(freq, TA, gA);
		//refract::mIce(freq, TB, gB);
		temp = zeros::secantMethod(formulaTemp, TA, TB, 0.00001);
		return temp;
	}
	catch (debug::xModelOutOfRange &)
	{
		return 0;
	}
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
