#include <iostream>
#include "../rtmath/rtmath.h"
#include "../mie/mie.h"
#include "../rtmath-base/rtmath-base.h"
#include <complex>
//#include <netcdf.h>

int main(int argc, char** argv)
{
	using namespace std;
	rtmath::debug::debug_preamble();
	cout << "Testing the basic mie library functions first\n";
	/*
	cout << "Testing wn for x = pi\n";
	mie::wnCalc wtest(3.14159);
	cout << "n\tW(1,n)\n";
	for (unsigned int i=0; i<10; i++)
	{
		complex<double> res = wtest.calc(i);
		cout << "w(" << i << ") = " << res << endl;
	}
	std::cin.get();
	cout << "Testing pi_n for theta = pi (mu = cos(pi))\n";
	mie::piNCalc pitest(3.14159);
	cout << "n\tpi(0,n)\n";
	for (unsigned int i=0; i<10; i++)
	{
		complex<double> res = pitest.calc(i);
		cout << "pi(" << i << ") = " << res << endl;
	}
	std::cin.get();
	
	cout << "Testing tau_n for theta = 0 (mu = 1)\n";
	mie::tauNCalc tautest(0.0);
	cout << "n\ttau(0,n)\n";
	for (unsigned int i=1; i<10; i++)
	{
		double res = tautest.calc(i);
		cout << "tau(" << i << ") = " << res << endl;
	}
	std::cin.get();
	
	cout << "Testing A_n for x = 1 (m = 1+0.33i)\n";
	mie::AnCalc Atest(2,complex<double>(1.1,0.63));
	cout << "n\tA(1,1+0.33i,n)\n";
	for (unsigned int i=0; i<16; i++)
	{
		complex<double> res = Atest.calc(i);
		cout << "A(" << i << ") = " << res << endl;
	}
	std::cin.get();
	
	std::complex<double> m(1.33,0);
	cout << "Testing the calculated an and bn\n";
	std::complex<double> an, bn;
	mie::abNCalc abncalc(m,2.0);
	for (unsigned int n=1;n<=10;n++)
	{
		abncalc.calc(n, an, bn);
		cout << n << "\t" << an << "\t" << bn << std::endl;
	}
	std::cin.get();
	cout << "Testing a few x points for the Qext code" << endl;

	mie::Qcalc q(m);
	for (double x=0.1;x<=1;x=x+0.1)
	{
		double Qe, Qa, Qs, g;
		q.calc(x,Qe,Qs,Qa, g);
		cout << Qe << std::endl;
	}
	
	std::cin.get();
	cout << "Testing the basic netCDF libraries\n";
	int fileid;
	int timeid;
	int voltageid;
	const char* desc = "The test attribute";
	nc_create("nctest.nc",0,&fileid);
	nc_def_dim(fileid, "time", NC_UNLIMITED, &timeid);
	nc_def_var(fileid, "voltage", NC_DOUBLE,1, &timeid, &voltageid);
	nc_put_att_text(fileid,voltageid,"desc",strlen(desc), desc);
	nc_enddef(fileid);
	double voltdata[] = {1.0, 2.0, 3.0, 3.5, 3.8, 2.5, 1.4, 1.2, 1.0, 0.5, 0.25};
	static size_t start[] = {0};
	static size_t count[] = {10};
	nc_put_vara_double(fileid, voltageid, start, count, voltdata);
	nc_close(fileid);
	cout << "netCDF file written!" << std::endl;
	std::cin.get();
	*/
	cout << "Gaussian quadrature test" << endl;
	cout << "Testing polynomial function y=4-x^2 with degrees 2-7" << endl;
	rtmath::polynomial x(1,1);
	rtmath::polynomial y = x*x * -1.0 + 4.0;
	for (unsigned int i=2; i<8; i++)
	{
		cout << i << "\t" << rtmath::quadrature::quad_eval_leg(-2.0,2.0,i,&y) << endl;
	}
	std::cin.get();

	return 0;
}
