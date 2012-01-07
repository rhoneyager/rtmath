#pragma once

#include<complex>
#include<map>

namespace mie {

	class wnCalc
	{
	public:
		wnCalc(double x);
		std::complex<double> calc(int n);
		~wnCalc(void);
	private:
		std::map<int, std::complex<double> > _Wn;
		double _x;
	};

}; // end mie

