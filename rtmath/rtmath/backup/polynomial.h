#pragma once

/* Polynomial header
   Code is used to provide support for representing polynomials in vectors
   Support for differentiation, integration, adding, recursion, ...
   Needed for Legendre polynomial stuff
   */

#include <map>
#include <complex>
#include <set>
#include "quadrature.h"

namespace rtmath {

	class polynomial : public evalfunction {
		public:
			polynomial();
			polynomial(unsigned int pow, double val);
//			polynomial(polynomial &orig);
			~polynomial();
			polynomial operator + (polynomial) const;
			polynomial operator + (double) const;
			polynomial operator - (double) const;
			polynomial operator - (polynomial) const;
			polynomial operator * (polynomial) const;
			polynomial operator * (double) const;
			polynomial operator ^ (unsigned int) const;
			polynomial deriv(unsigned int pow); // TODO: redo so it can be const
			void zeros(std::set<std::complex<double> > &zpts) const;
			double operator() (double) const;
			double& operator[] (const unsigned int); // TODO: redo for const
			double eval(double xval) const;
//			polynomial operator = (double);
			bool operator == (polynomial) const;
			bool operator == (double) const;
			bool operator != (double) const;
			bool operator != (polynomial) const;
			/*
			   Future operators:
			   +=, -=, *=, ^=
			   */
			void erase();
			void coeff(unsigned int pow, double val);
			double coeff(const unsigned int pow) const; // TODO: fix
			unsigned int maxPow() const;
			void truncate();
			void truncate(unsigned int pow);
			void print() const; // Error display
		private:
			std::map<unsigned int, double> _coeffs;
	};

};

