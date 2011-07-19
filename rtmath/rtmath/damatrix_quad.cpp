#include "Stdafx.h"
#include "damatrix_quad.h"
#include "matrixop.h"
#include "quadrature.h"
#include "damatrix.h"
#define _USE_MATH_DEFINES
#include <math.h>

namespace rtmath {
	namespace daint {
		std::shared_ptr<matrixop> outer_int(const mapid &valmap, 
			std::shared_ptr<damatrix> A, std::shared_ptr<damatrix> B)
		{
			// This is a namespace function that handles the outer integration loop/
			// Outer integration is from 0 to 2pi, dphi'
			// It varies A and B valmaps and calls the inner loop with Gaussian quadrature
			double a = 0.0;
			double b = 2.0 * M_PI;
			unsigned int deg = 7;
			// TODO: check the double / uint conversion
			unsigned int start = 3 * (unsigned int) (-1.0 - (double) deg / 2.0 + (double) deg * 
				(double) deg / 2.0);
			if (deg > 7) throw rtmath::debug::xBadInput();
			// res is the result that gets returned
			// resa is the corresponding matrixop
			matrixop resa(2,4,4);
			for (unsigned int i=start; i<start+(3*deg);i+=3)
			{
				double phip = ((b-a)/2.0 * rtmath::quadrature::_gaussian_lagrange_prepump[i+1])
					+ ((a+b)/2.0);
				resa = resa + *inner_int(valmap,phip,A,B);
			}
			resa = resa * ((b-a)/2.0);
			resa = resa * matrixop::diagonal(1.0/M_PI,2,4,4);
			std::shared_ptr<matrixop> res(new matrixop(resa));
			return res;
		}

		std::shared_ptr<matrixop> inner_int(const mapid &valmap, double phip, 
			std::shared_ptr<damatrix> A, std::shared_ptr<damatrix> B)
		{
			// This is a namespace function that handles the outer integration loop
			// Outer integration is 0 to 2pi, dphi'
			// It varies A and B valmaps and calls the inner loop with 
			//  gaussian quadrature
			unsigned int deg = 7;
			double a = 0.0, b = 1.0;
			unsigned int start = 3 * (unsigned int) (-1.0 - (double) deg / 2.0 + (double) deg * (double) deg / 2.0);
			if (deg > 7) throw rtmath::debug::xBadInput();
			matrixop resa(2,4,4);
			for (unsigned int i = start; i< start + (3*deg); i +=3)
			{
				double mup = ((b-a)/2.0 * rtmath::quadrature::_gaussian_lagrange_prepump[i+1]) 
					+ ((a+b)/2.0);
				// Assign Amap and Bmap the appropriate variables
				// From DA method, 
				mapid Amap(valmap.mu,mup,valmap.phi,phip), Bmap(mup,valmap.mun,phip,valmap.phin);
				//*res = *res + *A->eval(Amap) * *B->eval(Bmap) * mup;
				resa = resa + *A->eval(Amap) * *B->eval(Bmap) * mup;
			}
			resa = resa * ((b - a)/2.0);
			std::shared_ptr<matrixop> res(new matrixop(resa));
			return res;
		}
	}; // end namespace daint
}; // end namespace rtmath


