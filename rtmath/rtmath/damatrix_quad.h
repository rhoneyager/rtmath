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
		void outer_int(boost::shared_ptr<matrixop> res, const mapid &valmap, 
			boost::shared_ptr<damatrix> A, boost::shared_ptr<damatrix> B);
		boost::shared_ptr<matrixop> inner_int(
			boost::shared_ptr<damatrix> A, boost::shared_ptr<damatrix> B,
			const mapid &valmap, double phip);


		// The quadrature functions



	}; // end daint

}; // end rtmath
