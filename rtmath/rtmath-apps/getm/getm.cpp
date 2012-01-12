// backcalc_f is a program that calculates mie and rayleigh backscatter as a function of frequency and reports results in csv format. 
// Orientation does not matter because the particles are assumed to be either spherical or really small.

#include <iostream>
#include <fstream>
#include <string>
#include <set>
#include <vector>
#include <map>
#include <rtmath/rtmath.h>
#include <rtmath/mie/mie.h>
#include <boost/units/systems/si.hpp>
#include <memory>
#include <complex>
#define _USE_MATH_DEFINES
#include <cmath>

void doHelp();

int main(int argc, char** argv)
{
	using namespace std;
	// Parse the commands provided on the command line
	if (argc == 1) doHelp();
	rtmath::config::parseParams p(argc,argv);
	string oname;
	double f;
	double temp;
	bool mtab = false;
	if (argc != 3)
	{
		bool flag = false;
		flag = p.readParam("-t", temp);
		if (!flag) doHelp();
		flag = p.readParam("-f", f);
		if (!flag) doHelp();
		mtab = p.readParam("--mtab");
	} else {
		f = atof(argv[1]);
		temp = atof(argv[2]);
	}

	// Check program name to see if output should 
	// follow getm or genmtab (two for the code of one deal)
	if (string(argv[0]) == "rtmath-genmtab") mtab = true;

	std::complex<double> ref;
	rtmath::refract::mice(f,temp,ref);
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

	return 0;
}



void doHelp()
{
	using namespace std;
	cout << "rtmath-getm\n";
	cout << "Calculates the refractive index of ice (and water). Ryan version." << endl;
	cout << "Options:\n";
	cout << "If passed only two numbers, it interprets this as the old getm calling structure.\n";
	cout << "In this case, rtmath-getm f_GHz tempK\n";
	cout << "Otherwise, parameters are passed as follows:" << endl;
	cout << "-f (frequency GHz)\n";
	cout << "-t (temperature in K)\n";
	cout << "-m (UNSUPPORTED FOR NOW!!!)\n";
	cout << "\tMethod used for refractive index calculation (TODO). The method chosen also\n";
	cout << "\tdetermines if the refractive index is for water or ice. The methods are:\n";
	cout << "\t\tDefault: " << endl;
	cout << endl;
	exit(0);
}

