#pragma once

/* Polynomial header
   Code is used to provide support for representing polynomials in vectors
   Support for differentiation, integration, adding, recursion, ...
   Needed for Legendre polynomial stuff
   */

#include <map>
#include <set>
#include "quadrature.h"

namespace rtmath {

	class polynomial : public evalfunction {
		public:
			polynomial();
			polynomial(unsigned int pow, double val);
//			polynomial(polynomial &orig);
			~polynomial();
			polynomial operator + (polynomial);
			polynomial operator + (double);
			polynomial operator - (double);
			polynomial operator - (polynomial);
			polynomial operator * (polynomial);
			polynomial operator * (double);
			polynomial operator ^ (unsigned int);
			polynomial deriv(unsigned int pow);
			void zeros(std::set<double> &zpts);
			double operator() (double);
			double& operator[] (const unsigned int);
			double eval(double xval);
//			polynomial operator = (double);
			bool operator == (polynomial);
			bool operator == (double);
			bool operator != (double);
			bool operator != (polynomial);
			/*
			   Future operators:
			   +=, -=, *=, ^=
			   */
			void erase();
			void coeff(unsigned int pow, double val);
			double coeff(unsigned int pow);
			unsigned int maxPow();
			void truncate();
			void truncate(unsigned int pow);
			void print(); // Error display
		private:
			std::map<unsigned int, double> _coeffs;
	};

};

