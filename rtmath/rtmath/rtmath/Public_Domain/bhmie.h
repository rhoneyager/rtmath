#pragma once
/* The header for the modified Bohren and Huffman Mie Sphere
 * code. The basic function computes lots of quantities in 
 * one messy function. This header provides the interface in 
 * both raw and a pf-compatible manner. 
 */

#include <complex>
#include "../matrixop.h"
#include "../phaseFunc.h"
#include "../interpolatable.h"
#include "../ddscat/ddScattMatrix.h"
#include "../ddscat/ddOutputSingle.h"
#include "../da/daStatic.h"
#include "../coords.h"
namespace mie
{

	namespace bhmie
	{
		void bhmie(double x, const std::complex<double> &cxref, size_t nang, std::complex<double> cxs1[],
			std::complex<double> cxs2[], double &qext, double &qsca, double &qback, double &gsca);
	} ////
}     ////

		/*

		class Scalc : 
			public rtmath::scattMatrix
		{
		public:
			Scalc(const std::complex<double> &m, double x);
			~Scalc(void);
			void calc(double mu, double Snn[4][4], std::complex<double> Sn[4]);
		private:
			std::complex<double> _m;
			double _x;
		};

		class Qcalc
		{
		public:
			Qcalc(const std::complex<double> &m);
			void calc(double x, double &Qext, double &Qsca, double &Qabs, double &g);
		private:
			std::complex<double> _m;
		};

		class miePhaseFunc :
			public rtmath::phaseFunc
		{
		public:
			miePhaseFunc(double x, const std::complex<double> &m) : rtmath::phaseFunc()
			{
				_m = m;
				_x = x;
			}
			virtual ~miePhaseFunc(void) {}
			virtual std::shared_ptr<rtmath::matrixop> eval(double alpha) const;
		private:
			double _x;
			std::complex<double> _m;
		};


		class ddOutputMie : public rtmath::ddscat::ddOutputSingle
		{
			// Class contains the output of a single ddscat file
			// Doesn't quite inherit from daStatic. The files loaded by the ddOutput class
			// contain information on the scattering and emission matrices, so they are logically
			// two separate entries.
			// Note: ensemble providers inherit from this!
		public:
			// Constructors need to completely initialize, as I have const ddOutputSingle
			// shared pointers.
			// Read from ddscat file
			ddOutputMie(double freq, double psize, double span = 5.0);
			virtual ~ddOutputMie();
		private:
			void _init();
		public: // Made public for now so ensembles work. May just make that a friend class.
			ddOutputMie();
			//virtual void _insert(std::shared_ptr<const rtmath::ddscat::ddScattMatrix> &obj);
		};
	} // end bhmie

} // end mie

namespace rtmath
{
	namespace ddscat {
		class bhmieScattMatrix : public ddScattMatrix
		{
		public:
			bhmieScattMatrix(double freq, double theta, double reff);
			virtual ~bhmieScattMatrix();
		protected:
			double _reff;
			double _x;
			std::complex<double> _m;
		};
	} // end ddscat
} // end rtmath
*/