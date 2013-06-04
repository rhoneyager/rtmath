#pragma once

/* Quadrature routines
These routines are used to perform integration by quadrature methods.
They also contain precomputed weights and polynomial zeros.
Further polynomial zeros may be determined using the (completed)
polynomial class.
These functions should be called appropriately (set up a function pointer
or a class that provides an eval member and inherits from the function 
base class
*/
#include <set>
#include <cstddef> // size_t
#include <functional>

namespace rtmath {

	//extern const double _gaussian_lagrange_prepump[];

	namespace quadrature {
		double quad_eval_leg(double a, double b, unsigned int degree, 
			const std::function<double(double) > &f);
//		extern const double _gaussian_lagrange_prepump[];
//		void getQuadPtsLeg(size_t degree, std::set<double> &pts);

		typedef std::pair<double, double> ptWeight;
		void getQuadPtsLeg(size_t deg, std::set<ptWeight>&);
	}
}
