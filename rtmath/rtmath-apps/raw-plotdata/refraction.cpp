#include <string>
#include <iostream>
#include <memory>
#include <set>
#include <map>
#include <complex>
#include <cmath>
#define BOOST_TEST_DYN_LINK
#include "../rtmath/rtmath/error/error.h"
#include "../rtmath/rtmath/refract.h"

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(refraction);

using namespace std;
using namespace rtmath;
using namespace rtmath::debug;
//using namespace rtmath::ddscat;

BOOST_AUTO_TEST_CASE(e_f_data)
{
	const double T = 260; // K
	const double freq = 35.6; // GHz
	complex<double> Matm(1,0);
	complex<double> Mice;
	rtmath::refract::mice(freq,T,Mice);

	complex<double> e_atm = Matm * Matm;
	complex<double> e_ice = Mice * Mice;

	ofstream out("e-260K-35.6GHz.dat");
	out << "fraction\te_atm_r\te_atm_i\te_ice_r\te_ice_imag\tMice_real\tMice_imag\te_r\te_i\tM-re\tM_im" << endl;
	for (double f=0; f<=1.0; f += 0.01)
	{
		complex<double> ea = f * (e_ice - complex<double>(1.,0)) / (e_ice + complex<double>(2.,0));
		complex<double> eb = (1.-f) * (e_atm - complex<double>(1.,0)) / (e_atm + complex<double>(2.,0));

		complex<double> fact = ea+eb;
		complex<double> e = (complex<double>(2.,0) * fact + complex<double>(1.,0)) 
			/ (complex<double>(1.,0) - fact);

		out << f << "\t" << e_atm.real() << "\t" << e_atm.imag() << "\t" << e_ice.real() << "\t" << 
			e_ice.imag() << "\t" << Mice.real() << "\t" << Mice.imag() << "\t" << 
			e.real() << "\t" << e.imag() << "\t" << 
			sqrt(e).real() << "\t" << sqrt(e).imag() << endl;
	}
}




BOOST_AUTO_TEST_SUITE_END();

