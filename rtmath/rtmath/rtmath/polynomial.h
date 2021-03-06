#pragma once
#include "defs.h"

#include <map>
#include <iostream>
#include <cstdarg>
#include "quadrature.h"

namespace rtmath {

	/** \brief Polynomial class
	*
	* Code is used to provide support for representing polynomials in vectors
	* Support for differentiation, integration, adding, recursion, ...
	* Needed for Legendre polynomial stuff
	**/
	class DLEXPORT_rtmath_core polynomial {
		public:
			polynomial(); 
			polynomial(unsigned int pow, double val); 
			polynomial(size_t ndims, ...); 
			polynomial(const polynomial &orig);
			virtual ~polynomial(void);

			void toDoubleArray(double *target) const;
			void fromDoubleArray(size_t maxdeg, const double *source); // maxdeg is highest degree

			polynomial* clone() const;
			polynomial & operator = (const polynomial&);
			polynomial & operator = (double); // Assignment from double (array impossible, as degree of polynomial will be unknown)
			bool operator == (const polynomial&) const;
			bool operator != (const polynomial&) const;
			bool approxEq(const polynomial&) const;
			bool approxEq(double) const;
			bool operator == (double) const;
			bool operator != (double) const;

			virtual double eval(double val) const;
			virtual double operator() (double val) const; // Evaluate polynomial at specified value
			// Disabled, as it messes with arrays of polynomials and is unclean
			//double& operator[] (unsigned int deg) const; // Get coefficient for specified degree
			double coeff(unsigned int deg) const; // Get coefficient for specified degree
			void coeff(unsigned int deg, double val); // Set coefficient for specified degree

			polynomial operator + (const polynomial&) const;
			polynomial operator - (const polynomial&) const;
			polynomial operator * (const polynomial&) const;
			polynomial operator + (double) const;
			polynomial operator - (double) const;
			polynomial operator * (double) const;
			polynomial operator ^ (unsigned int) const;

			polynomial& operator += (const polynomial&);
			polynomial& operator -= (const polynomial&);
			polynomial& operator *= (const polynomial&);
			polynomial& operator += (double);
			polynomial& operator -= (double);
			polynomial& operator *= (double);
			polynomial& operator ^= (unsigned int);

			polynomial deriv(unsigned int deg = 1) const;
			//polynomial integ(unsigned int deg) const;

			//void zeros(std::multiset<std::complex<double> > &zpts) const;
			//void zeros(std::complex<double> *zeros) const;
			/*
			inline std::multiset<std::complex<double> > zeros() const
			{
			}
			*/

			void erase();
			void print() const;
			void truncate();
			void truncate(unsigned int pow);
			unsigned int maxPow() const;

		private:
			std::map<unsigned int, double> _coeffs;
			std::string _var;
			void _init();
	};

};


DLEXPORT_rtmath_core std::ostream & operator<<(std::ostream &stream, const rtmath::polynomial &ob);
//std::istream &operator>>(std::istream &stream, rtmath::polynomial &ob);


