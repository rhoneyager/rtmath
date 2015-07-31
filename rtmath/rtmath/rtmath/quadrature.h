#pragma once
#include "defs.h"

#include <set>
#include <cstddef> // size_t
#include <functional>

namespace rtmath {

	//extern const double _gaussian_lagrange_prepump[];

	/** \brief Quadrature routines
	*
	* These routines are used to perform integration by quadrature methods.
	* They also contain precomputed weights and polynomial zeros.
	* Further polynomial zeros may be determined using the (completed)
	* polynomial class.
	* 
	* These functions should be called appropriately (set up a function pointer
	* or a class that provides an eval member and inherits from the function 
	* base class
	**/
	namespace quadrature {
		double DLEXPORT_rtmath_core quad_eval_leg(double a, double b, unsigned int degree, 
			const std::function<double(double) > &f);
//		extern const double _gaussian_lagrange_prepump[];
//		void getQuadPtsLeg(size_t degree, std::set<double> &pts);

		typedef std::pair<double, double> ptWeight;
		void DLEXPORT_rtmath_core getQuadPtsLeg(size_t deg, std::set<ptWeight>&);

		/// Trapezoid rule integration, with specified dN
		template <class T>
		T integrateTrapezoid2(const std::function<double(double)> &f, T low, T high, T dN)
		{
			T res = 0;
			T vLast = f(low);
			for (T val = low; val < high; val += dN) {
				T vNext = f(val+dN);
				res += (T) ((vNext + vLast) / (T) 2.);
				vLast = vNext;
			}
			res *= dN;
			return res;
		}


		/// Trapezoid rule integration, autocalculate dN
		template <class T>
		T integrateTrapezoid(const std::function<double(double)> &f, T low, T high, size_t n)
		{
			T dN = (high - low) / (T) n;
			return integrateTrapezoid2<T>(f, low, high, dN);
		}

	}
}

