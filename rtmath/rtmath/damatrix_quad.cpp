#include "Stdafx.h"
#include "damatrix_quad.h"
#include "matrixop.h"
#include "quadrature.h"
#include "damatrix.h"
#define _USE_MATH_DEFINES
#include <math.h>

namespace rtmath {
	namespace daint {
		unsigned int deg = 7;

		std::shared_ptr<matrixop> outer_int(const mapid &valmap, 
			std::shared_ptr<damatrix> A, std::shared_ptr<damatrix> B)
		{
			// This is a namespace function that handles the outer integration loop/
			// Outer integration is from 0 to 2pi, dphi'
			// It varies A and B valmaps and calls the inner loop with Gaussian quadrature
			double a = 0.0;
			//double b = 2.0 * M_PI; // Oops - I had switched outer_int and inner_int's bounds
			double b = 1.0;
			//unsigned int deg = 7;
			
			unsigned int start = 3 * (unsigned int) ( ( (deg * deg) - deg) / 2);
			//unsigned int start = 3 * (unsigned int) (-1.0 * (double) deg / 2.0 + (double) deg * (double) deg / 2.0);
			if (deg > 7) throw rtmath::debug::xBadInput();
			if (deg == 0) throw rtmath::debug::xBadInput();
			// res is the result that gets returned
			// resa is the corresponding matrixop
			matrixop resa(2,4,4);
			for (unsigned int i=start; i<start+(3*deg);i+=3)
			{
				double phip = ((b-a)/2.0 * rtmath::quadrature::_gaussian_lagrange_prepump[i+1])
					+ ((a+b)/2.0);
				resa = resa + (*inner_int(valmap,phip,A,B) * rtmath::quadrature::_gaussian_lagrange_prepump[i+2]);
			}
			resa = resa * ((b-a)/2.0);
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
			//unsigned int deg = 7;
			double a = 0.0;
			double b = 2.0 * M_PI;
			unsigned int start = 3 * (unsigned int) ( ( (deg * deg) - deg) / 2);
			//unsigned int start = 3 * (unsigned int) (-1.0 * (double) deg / 2.0 + (double) deg * (double) deg / 2.0);
			if (deg > 7) throw rtmath::debug::xBadInput();
			if (deg == 0) throw rtmath::debug::xBadInput();
			matrixop resa(2,4,4);
			for (unsigned int i = start; i< start + (3*deg); i +=3)
			{
				double mup = ((b-a)/2.0 * rtmath::quadrature::_gaussian_lagrange_prepump[i+1]) 
					+ ((a+b)/2.0);
				double muweight = rtmath::quadrature::_gaussian_lagrange_prepump[i+2];
				// Assign Amap and Bmap the appropriate variables
				// From DA method, 
				// Note: I'll do it this way to allow me to change mapid's definition later
				mapid Amap = valmap; 
				Amap.mun = mup;
				Amap.phin = phip;
				mapid Bmap = valmap;
				Bmap.mu = mup;
				Bmap.phi = phip;
				//mapid Amap(valmap.mu,mup,valmap.phi,phip), Bmap(mup,valmap.mun,phip,valmap.phin);
				//*res = *res + *A->eval(Amap) * *B->eval(Bmap) * mup;
				resa = resa + *A->eval(Amap) * *B->eval(Bmap) * muweight;
			}
			resa = resa * ((b - a)/2.0);
			std::shared_ptr<matrixop> res(new matrixop(resa));
			return res;
		}
	}; // end namespace daint
}; // end namespace rtmath


