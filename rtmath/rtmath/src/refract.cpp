// Code segment directly based on Liu's code. Rewritten in C++ so that it may be 
// compiled in an MSVC environment

#include "../rtmath/Stdafx.h"
#include <cmath>
#include <complex>
#include <fstream>
#include "../rtmath/refract.h"

// Ice complex refractive index
// based on Christian Matzler (2006)
void rtmath::refract::mice(double f, double t, std::complex<double> &m)
{
	double er = 0;
	if (t>243.0)
		er = 3.1884+9.1e-4*(t-273.0);
	else
		er = 3.1611+4.3e-4*(t-243.0);
	// Imaginary part
	double theta = 300.0/(t-1.0);
	double alpha = (0.00504+0.0062*theta)*exp(-22.1*theta);
	double dbeta = exp(-9.963+0.0372*(t-273.16));
	const double B1 = 0.0207;
	const double B2 = 1.16e-11;
	const double b = 335;
	double betam = B1/t*exp(b/t)/pow((exp(b/t)-1.0),2)+B2*f*2.0;
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

