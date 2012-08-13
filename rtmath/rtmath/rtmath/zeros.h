#pragma once
#include <complex>

#include "error/error.h"
#include "polynomial.h"
#include "quadrature.h"

namespace rtmath {

	namespace zeros {
		//double findzero(double a, double b, double (*evalfunc)(double) );
		double findzero(double a, double b, evalfunction* evaltarget);

		template<class T>
		void complexSecant(const T &f, 
			std::complex<double> guess_a, std::complex<double> guess_b,
			std::complex<double> &zero, double eps = 0.0001, size_t maxIter = 50)
		{
			// Secant method is defined by recurrance
			// xn = x_(n-1) - f(x_(n-1)) * (x_(n-1) - x_(n-2)) / (f(x_(n-1)) - f(x_(n-2)))
			using namespace std;
			complex<double> xn = guess_a, xn1 = guess_b, xn2;
			complex<double> fxn1, fxn2;
			size_t i=0;
			do {
				xn2 = xn1;
				xn1 = xn;

				fxn1 = f(xn1);
				fxn2 = f(xn2);

				xn = xn1 - fxn1 * (xn1 - xn2) / (fxn1 - fxn2);
			} while ( (abs(xn-xn1) > eps) && (i++ < maxIter));

			if (i == maxIter) throw debug::xModelOutOfRange(xn.real());
			zero = xn;
		}

	}; // end namespace zeros

}; // end namespace rtmath

