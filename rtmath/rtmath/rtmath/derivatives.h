#pragma once
#error "Unimplemented header"
#include "defs.h"

namespace rtmath
{
	namespace derivatives
	{
		/**
		\brief Provides the nth derivative with a given set of sample points

		This template provides the capability of finding the nth central 
		derivative of an arbitrary function (func). Func must provide 
		operator(), allowing for evaluation with either classes or 
		lambda functions.

		The kernel size, k, represents the number of points around the 
		central point that contribute to the derivative calculation.

		Furthermore, the template accepts different result types (generally 
		restricted to classes / types that obey the standard C++ 
		arithmetic operations).

		\param T The floating point type used for evaluations and returns
		\param sample_points The number of points used in the finite difference kernel
		\param dx The step size for the sample points
		\param x The point at which the derivative should be evaluated
		\return The result of the derivative operation

		\todo Implement this.
		**/
		template <typename T, class Function_type>
		T ERR_UNIMPLEMENTED derivative_base(Function_type func, const std::vector<T> &sample_points, T dx, T x)
		{
#pragma message("rtmath::derivatives::derivative is unimplemented")
			throw; // Unimplemented
			return 0;
		}

		/**
		Provides a central derivative
		*/

	}
}
