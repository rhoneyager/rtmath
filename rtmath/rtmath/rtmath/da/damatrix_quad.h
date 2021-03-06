#pragma once
// This header provides all of the quadrature functions for damatrices.
// This is based on the quadrature code in the quadrature class, but 
// that class cannot handle quadrature of a matrix. Quadrature is 
// performed following Lacis and Hansen (1974), or one of Hansen's many 
// other papers. Liou uses the identical algorithm, too.

#include "../matrixop.h"
#include <memory>
#include "damatrix.h"

namespace rtmath {
	class damatrix;
	namespace daint {
		extern unsigned int deg;
		std::shared_ptr<const matrixop> outer_int(const mapid &valmap, 
			std::shared_ptr<rtmath::damatrix> A, std::shared_ptr<rtmath::damatrix> B);
		std::shared_ptr<const matrixop> inner_int(const mapid &valmap, double phip, 
			std::shared_ptr<rtmath::damatrix> A, std::shared_ptr<rtmath::damatrix> B);
	}; // end namespace daint
}; // end namespace rtmath

