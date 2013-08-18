#pragma once
#include "../defs.h"

namespace rtmath {
	class polynomial;
	namespace recPolys {
		/**
		 \brief Define a pure virtual class that provides structure to the set of 
		 recursive polynomials. 
		 
		 The recursive polynomials do not instiantize.
		 Instead, static member functions produce the polynomials, which are 
		 then added to a cache. When a desired polynomial is requested, a 
		 clone of the object is returned.
		
		 \note Due to limitaions with static data members, the cached data store
		 must be defined in each polynomial.

		 \todo Try to replace the derived classes with the boost::math functions.
		 **/
		class DLEXPORT_rtmath_core recPoly {
			public:
				recPoly() {}
				virtual ~recPoly() {}
				/// Accessor function to get the polynomial of a certain degree.
				virtual void get(unsigned int degree, polynomial &res) const = 0;
				/*
				inline virtual polynomial get(unsigned int degree) const
				{
					polynomial res;
					get(degree, res);
					return res;
				}
				*/
		};
	} // end namespace recPolys
} // end namespace rtmath

