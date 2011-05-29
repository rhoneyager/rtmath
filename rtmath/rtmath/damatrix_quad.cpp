#include "Stdafx.h"
#include "damatrix.h"
#include "damatrix_quad.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "../rtmath-base/quadrature.h"

namespace rtmath {
	
void daint::outer_int(boost::shared_ptr<matrixop> res, const mapid &valmap, 
		boost::shared_ptr<damatrix> A, boost::shared_ptr<damatrix> B)
	{
		// This is a namespace function that handles the outer integration loop
		// Outer integration is 0 to 2pi, dphi'
		// It varies A and B valmaps and calls the inner loop with 
		//  gaussian quadrature
		double a = 0.0;
		double b = 2.0 * M_PI;
		unsigned int deg = 7;
		// TODO: check the double / uint conversion
		unsigned int start = 3 * (unsigned int) (-1.0 - (double) deg / 2.0 + (double) deg * (double) deg / 2.0);
		if (deg > 7) throw;
		// res is already defined
		for (unsigned int i = start; i< start + (3*deg); i +=3)
		{
			double phip = ((b-a)/2.0 * rtmath::_gaussian_lagrange_prepump[i+1]) 
				+ ((a+b)/2.0);
			*res = *res + *inner_int(A,B,valmap,phip);
		}
		*res = *res * ((b - a)/2.0);
	}

	boost::shared_ptr<matrixop> daint::inner_int(
		boost::shared_ptr<damatrix> A, boost::shared_ptr<damatrix> B,
		const mapid &valmap, double phip)
	{
		// This is a namespace function that handles the outer integration loop
		// Outer integration is 0 to 2pi, dphi'
		// It varies A and B valmaps and calls the inner loop with 
		//  gaussian quadrature
		unsigned int deg = 7;
		double a = 0.0, b = 1.0;
		unsigned int start = 3 * (unsigned int) (-1.0 - (double) deg / 2.0 + (double) deg * (double) deg / 2.0);
		if (deg > 7) throw;
		boost::shared_ptr<matrixop> res(new matrixop(A->size()));
		for (unsigned int i = start; i< start + (3*deg); i +=3)
		{
			double mup = ((b-a)/2.0 * rtmath::_gaussian_lagrange_prepump[i+1]) 
				+ ((a+b)/2.0);
			// Assign Amap and Bmap the appropriate variables
			// From DA method, 
			mapid Amap(valmap.mu,mup,valmap.phi,phip), Bmap(mup,valmap.mun,phip,valmap.phin);
			*res = *res + *A->eval(Amap) * *B->eval(Bmap) * mup;
		}
		*res = *res * ((b - a)/2.0);
		return res;
	}


}; // end rtmath
