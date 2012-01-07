#include "../rtmath/Stdafx.h"
#include "../rtmath/rtmath.h"

namespace mie {

	// Remember, m is a complex refractive index

	AnCalc::AnCalc(double x, const std::complex<double> &m)
	{
		using namespace std;
		this->x = x;
		this->m = m;
		// Calculate the zero value
		complex<double> res = m;
		res *= complex<double>(x,0);
		res = cos(res)/sin(res);
		this->An.push_back(res);
		// Yeah, this was an awkward way of doing this...
	}

	std::complex<double> AnCalc::calc(unsigned int n)
	{
		// This requires recursion, so see if already precalculated
		using namespace std;
		if(An.size() > n) return An[n];
		// Calculation is necessary.
		for (unsigned int i = (unsigned int) An.size(); i <= n; i++)
		{
			complex<double> res(0.0,0.0);
			res = (complex<double>(i,0))/(x*m) - An[i-1];
			res = complex<double>(1.0,0.0) / res;
			res += -1.0 * (complex<double>(i,0.0)/(x*m));
			An.push_back(res);
		}
		return An[n];
	}


}; // end mie

