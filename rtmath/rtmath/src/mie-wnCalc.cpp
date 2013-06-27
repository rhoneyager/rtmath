#define EXPORTING_RTMATH
#include "../rtmath/mie/mie-wnCalc.h"
#include <cmath>

namespace rtmath {
	namespace mie {

		wnCalc::wnCalc(double sizep) : _sizep(sizep)
		{
			this->_Wn[-1] = std::complex<double>(cos(sizep),-1.0*sin(sizep));
			this->_Wn[0] = std::complex<double>(sin(sizep),cos(sizep));
		}

		std::complex<double> wnCalc::calc(int n) const
		{
			// Calculate the appropriate Wn based on the known value of x
			using namespace std;
			// First, see if the table has this value already calculated
			if (_Wn.count(n)) return _Wn[n];
			// If not, calculate it and populate the vector
			// Don't use recursion, as it's slow and kills the stack
			// Find the max. n value (they should be continuous)
			// This is the map.size - 2 (-1, 0, 1, 2, 3, ...)
			int maxN = (int) _Wn.size() - 2;

			for (int i = maxN + 1; i <= n; i++)
			{
				// Calculate each value
				// i is an index, not imaginary
				_Wn[i] = ((2.0*i - 1.0) / _sizep) * _Wn[i-1] - _Wn[i-2];
			}
			return _Wn[n];
		}


	}
}
