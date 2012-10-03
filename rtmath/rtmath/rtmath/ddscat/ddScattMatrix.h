#pragma once
//#include <iostream>
#include <complex>
#include "../matrixop.h"

#include <boost/shared_ptr.hpp>

namespace rtmath
{
	namespace ddscat
	{
		// This is now a base class because there are two ways for
		// specifying scattering: the complex scattering amplitude 
		// matrix or the scattering phase matrix. ddscat intermediate 
		// output gives the complex matrix which is more useful. The 
		// P matrix is harder to derive from, but is found in the 
		// .avg files and in tmatrix code.

		enum scattMatrixType
		{
			F,
			P
		};

		class ddScattMatrix
		{
		public:
			ddScattMatrix();
			virtual ~ddScattMatrix();
			ddScattMatrix & operator = (const ddScattMatrix&); // Assignment needed due to arrays

			virtual matrixop mueller() const;
			double pol() const;
			void pol(double);
		//protected:
			mutable boost::shared_ptr<matrixop> _Pnn;
			double _pol;
			virtual scattMatrixType id() const { return P; }
		};

		class ddScattMatrixF : public ddScattMatrix
		{
		public:
			// Needs frequency (GHz) and phi (degrees) for P and K calculations
			ddScattMatrixF(double freq = 0, double phi = 0);
			virtual ~ddScattMatrixF();
			ddScattMatrixF & operator = (const ddScattMatrixF&);
			virtual scattMatrixType id() const { return F; }
			// matrixop extinction() const;
			virtual matrixop mueller() const;
			void setF(const std::complex<double> fs[2][2]);
			//void setF(std::istream &lss); // Include this higher up
			matrixop getF() const;
			matrixop getS() const;
		//protected:
			void _calcS() const;
			void _calcP() const;
			//boost::shared_array<std::complex<double> > _f, _s;
			boost::shared_ptr<matrixop> _fRe, _fIm; // Should store as shared_array
			boost::shared_ptr<matrixop> _sRe, _sIm;
			double _freq, _phi;
		};

		/* class ddScattMatrixS : public ddScattMatrix
		{
		}; */

		class ddScattMatrixP : public ddScattMatrix
		{
		public:
			ddScattMatrixP() {}
			virtual ~ddScattMatrixP() {}
			ddScattMatrixP & operator = (const ddScattMatrixP&);
			virtual scattMatrixType id() const { return P; }
			void setP(const matrixop&);
			matrixop getP() const;
		};
	}
}

