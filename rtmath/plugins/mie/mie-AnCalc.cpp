#include "mie-AnCalc.h"
#include <iostream>

namespace rtmath {
	namespace plugins {
		namespace mie {

			// Remember, m is a complex refractive index

			AnCalc::AnCalc(double sizep, const std::complex<double> &m)
				: sizep(sizep), m(m)
			{
				using namespace std;
				// Calculate the zero value
				complex<double> res = m;
				res *= complex<double>(sizep, 0);
				res = cos(res) / sin(res);
				this->An.push_back(res);
				// Yeah, this was an awkward way of doing this...
			}

			std::complex<double> AnCalc::calc(size_t n) const
			{
				// This requires recursion, so see if already precalculated
				using namespace std;
				if (An.size() > n) return An[n];
				An.reserve(n);
				// Calculation is necessary.
				for (size_t i = An.size(); i <= n; i++)
				{
					complex<double> res(0.0, 0.0);
					res = (complex<double>((double)i, 0)) / (sizep*m) - An[i - 1];
					res = complex<double>(1.0, 0.0) / res;
					res += -1.0 * (complex<double>((double)i, 0.0) / (sizep*m));
					An.push_back(std::move(res));
				}
				//std::cerr << "   An n " << n << " m " << m << " sizep " << sizep << std::endl;
				//std::cerr << "   An[n] " << An[n] << std::endl;
				return An[n];
			}


		}
	}
}
