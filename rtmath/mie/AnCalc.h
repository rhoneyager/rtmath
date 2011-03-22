#pragma once

#include<vector>
#include<complex>

namespace mie {

	class AnCalc
	{
	public:
		std::complex<double> calc(unsigned int n);
		AnCalc(double x, const std::complex<double> &m);
		std::vector< std::complex<double> > An;
	private:
		double x;
		std::complex<double> m;
	};

}; // end mie


