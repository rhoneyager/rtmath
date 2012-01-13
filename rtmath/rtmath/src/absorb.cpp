#include <cmath>
#include "../rtmath/absorb.h"

namespace rtmath {
	namespace absorb {

		absorb::~absorb()
		{
			// Does nothing. Entry exists to ensure 
			// that vtable builds here.
		}

		double absorb::T(double tau)
		{
			// Quick and easy return of 
			// the transmittance
			return exp(-tau);
		}

		double absorb::_wvtofreq(double wvnum)
		{
			// Really should use full speed of light
			// TODO: replace definition
			return wvnum * 3.e8;
		}

		double collide::tau(double wvnum) const
		{
			return 0.0;
		}
	};
}; // end namespace rtmath

