#define EXPORTING
#include "../rtmath/mie/mie-tauNCalc.h"
#include "../rtmath/mie/mie-piNCalc.h"

namespace rtmath {
	namespace mie {

		tauNCalc::tauNCalc(double mu) : mu(mu)
		{
			pin = boost::shared_ptr<piNCalc>(new piNCalc(mu));
		}

		double tauNCalc::calc(size_t n) const
		{
			double res;
			res = n * mu * pin->calc(n) - (n + 1.0) * pin->calc(n-1);
			return res;
		}

	}
}
