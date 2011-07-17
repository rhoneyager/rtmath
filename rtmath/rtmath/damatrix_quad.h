/* damatrix-quad.h
Doubling-Adding quadrature routines
These are needed because rtmath-base provides quadrature only for doubles
and functions that return double.
I can't extend to just matrixop because it has no eval() function, and rtmath
provides damatrix.
*/
#pragma once


namespace rtmath {

	namespace daint {

		// Note here that A and B are not const, since they memorize pre-calculated values
		void outer_int(matrixop &res, const mapid &valmap, 
			damatrix* A, damatrix* B);
		std::shared_ptr<matrixop> inner_int(
			damatrix* A, damatrix* B,
			const mapid &valmap, double phip);


		// The quadrature functions



	}; // end daint

}; // end rtmath
