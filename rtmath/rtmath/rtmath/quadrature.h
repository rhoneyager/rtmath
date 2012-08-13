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
#include <map>
#include <set>

namespace rtmath {

	//extern const double _gaussian_lagrange_prepump[];

	class evalfunction // an evaluatable function (allows class evaluation)
	{
	public:
		evalfunction();
		virtual ~evalfunction();
		virtual double eval(double val) const = 0;
		virtual double operator() (double) const = 0;
	};

	namespace quadrature {
		double quad_eval_leg(double a, double b, unsigned int degree, const evalfunction *f);
		extern const double _gaussian_lagrange_prepump[];

		void getQuadPtsLeg(size_t degree, std::set<double> &pts);
	};
}; // end namespace rtmath
