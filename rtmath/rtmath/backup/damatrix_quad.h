#pragma once
// This header provides all of the quadrature functions for damatrices.
// This is based on the quadrature code in the quadrature class, but 
// that class cannot handle quadrature of a matrix. Quadrature is 
// performed following Lacis and Hansen (1974), or one of Hansen's many 
// other papers. Liou uses the identical algorithm, too.

#include "matrixop.h"
#include <memory>
#include "damatrix.h"

namespace rtmath {
	namespace daint {
		extern unsigned int deg;
		std::shared_ptr<matrixop> outer_int(const mapid &valmap, 
			std::shared_ptr<damatrix> A, std::shared_ptr<damatrix> B);
		std::shared_ptr<matrixop> inner_int(const mapid &valmap, double phip, 
			std::shared_ptr<damatrix> A, std::shared_ptr<damatrix> B);
	}; // end namespace daint
}; // end namespace rtmath

