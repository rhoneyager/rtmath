#include "wnCalc.h"
#include <cmath>

namespace mie {

	wnCalc::wnCalc(double x)
	{
		_x = x;
		this->_Wn[-1] = std::complex<double>(cos(x),-1.0*sin(x));
		this->_Wn[0] = std::complex<double>(sin(x),cos(x));
	}


	wnCalc::~wnCalc(void)
	{
		// Do nothing
	}

	std::complex<double> wnCalc::calc(int n)
	{
		// Calculate the appropriate Wn based on the known value of x
		using namespace std;
		// First, see if the table has this value already calculated
		if (_Wn.count(n) > 0) return _Wn[n];
		// If not, calculate it and populate the vector
		// Don't use recursion, as it's slow and kills the stack
		// Find the max. n value (they should be continuous)
		// This is the map.size - 2 (-1, 0, 1, 2, 3, ...)
		int maxN = _Wn.size() - 2;

		for (int i = maxN + 1; i <= n; i++)
		{
			// Calculate each value
			// i is an index, not imaginary
			_Wn[i] = ((2.0*i - 1.0) / _x) * _Wn[i-1] - _Wn[i-2];
		}
		return _Wn[n];
	}


}; // end mie

