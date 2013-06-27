#define EXPORTING_RTMATH
#include "../rtmath/mie/mie-piNCalc.h"

namespace rtmath {
	namespace mie {

		piNCalc::piNCalc(double mu)
			: mu(mu)
		{
			pin.push_back(0);
			pin.push_back(1);
		}

		double piNCalc::calc(size_t n) const
		{
			if (pin.size() > n) return pin[n];
			pin.reserve(n);
			for (size_t i=pin.size(); i<=n; i++)
			{
				double res;
				res = ((2.0*i - 1.0) / (i - 1.0)) * mu * pin[i-1]
				- ((i) / (i - 1.0)) * pin[i-2];
				pin.push_back(res);
			}
			return pin[n];
		}

	}
}

