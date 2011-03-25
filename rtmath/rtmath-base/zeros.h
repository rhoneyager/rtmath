#pragma once

#include "polynomial.h"
#include "quadrature.h"

namespace rtmath {

	namespace zeros {
		//double findzero(double a, double b, double (*evalfunc)(double) );
		double findzero(double a, double b, evalfunction* evaltarget);
	}; // end namespace zeros

}; // end namespace rtmath

