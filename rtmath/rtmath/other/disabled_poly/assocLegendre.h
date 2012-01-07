#pragma once

#include <set>
#include "polynomial.h"
#include "legendre.h"

namespace rtmath {
	namespace recPoly {
		class assocLegendre
		{
			//TODO: optimise the functions to remember the precalced
			// associated legendre parts!
			public:
				static double eval(double x, unsigned int l, int m);
			private:
				static rtmath::legendre _legparts;
			public:
				virtual void get(unsigned int rank, polynomial &res) const;
			private:
				static std::vector<polynomial> _cache;
		};
	};

};

