#include <iostream>
#include "../rtmath/rtmath.h"
#include "../mie/mie.h"
#include <complex>

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
	cout << "Testing pi_n for theta = 0 (mu = 1)\n";
	mie::piNCalc pitest(0.0);
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
	mie::AnCalc Atest(1,complex<double>(1,0.33));
	cout << "n\tA(1,1+0.33i,n)\n";
	for (unsigned int i=0; i<10; i++)
	{
		complex<double> res = Atest.calc(i);
		cout << "A(" << i << ") = " << res << endl;
	}
	std::cin.get();
	*/
	cout << "Testing a few x points for the Qext code" << endl;
	std::complex<double> m(1.33,0);
	mie::Qcalc q(m);
	for (double x=0.1;x<10;x=x+0.01)
	{
		double Qe, Qa, Qs;
		q.calc(x,Qe,Qs,Qa);
		cout << Qe << std::endl;
	}
	std::cin.get();
	return 0;
}
