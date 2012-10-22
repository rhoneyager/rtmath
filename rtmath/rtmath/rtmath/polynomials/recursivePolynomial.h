#pragma once
#include "../polynomial.h"


namespace rtmath {
	namespace recPolys {
		// Define a pure virtual class that provides structure to the set of 
		// recursive polynomials. The recursive polynomials do not instiantize.
		// Instead, static member functions produce the polynomials, which are 
		// then added to a cache. When a desired polynomial is requested, a 
		// clone of the object is returned.
		//
		// Note: due to limitaions with static data members, the cached data store
		// must be defined in each polynomial.
		class recPoly {
			public:
				recPoly() {}
				virtual ~recPoly();
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
	}; // end namespace recPolys
}; // end namespace rtmath

