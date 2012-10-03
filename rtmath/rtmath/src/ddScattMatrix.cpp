#include "../rtmath/Stdafx.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <complex>
#include "../rtmath/matrixop.h"
#include "../rtmath/error/error.h"
#include "../rtmath/ddscat/ddScattMatrix.h"
#include "../rtmath/units.h"
#include "../rtmath/phaseFunc.h"
//#include "../rtmath/coords.h"

namespace {
	void complexify(const rtmath::matrixop &re, const rtmath::matrixop &im, std::complex<double> *res)
	{
		if (re.maxSize() != im.maxSize()) throw;
		size_t maxSize = re.maxSize();
		for (size_t i=0; i< maxSize; i++)
		{
			res[i] = std::complex<double>(re.getIndex(i), im.getIndex(i));
		}
	}
}

namespace rtmath {
	namespace ddscat {

		ddScattMatrix::ddScattMatrix()
		{
			_Pnn = boost::shared_ptr<matrixop>(new matrixop(2,4,4));
			_pol = 0;
		}

		ddScattMatrix::~ddScattMatrix()
		{
		}

		ddScattMatrix& ddScattMatrix::operator = (const ddScattMatrix &rhs)
		{
			if (this == &rhs) return *this; // self-assignment check

			_Pnn = boost::shared_ptr<matrixop>(new matrixop(*(rhs._Pnn)));

			return *this;
		}

		void ddScattMatrix::pol(double npol)
		{
			_pol = npol;
		}

		double ddScattMatrix::pol() const
		{
			return _pol;
		}

		matrixop ddScattMatrix::mueller() const
		{
			return *_Pnn;
		}

		ddScattMatrixF::ddScattMatrixF(double freq, double phi)
		{
			_freq = freq;
			_phi = phi;
			_fRe = boost::shared_ptr<matrixop>(new matrixop(2,2,2));
			_fIm = boost::shared_ptr<matrixop>(new matrixop(2,2,2));
			_sRe = boost::shared_ptr<matrixop>(new matrixop(2,2,2));
			_sIm = boost::shared_ptr<matrixop>(new matrixop(2,2,2));
		}

		ddScattMatrixF::~ddScattMatrixF()
		{
		}

		ddScattMatrixF & ddScattMatrixF::operator = (const ddScattMatrixF &rhs)
		{
			if (this == &rhs) return *this; // self-assignment check

			_freq = rhs._freq;
			_phi = rhs._phi;

			_Pnn = boost::shared_ptr<matrixop>(new matrixop(*(rhs._Pnn)));

			_fRe = boost::shared_ptr<matrixop>(new matrixop(*(rhs._fRe)));
			_fIm = boost::shared_ptr<matrixop>(new matrixop(*(rhs._fIm)));

			return *this;
		}

		ddScattMatrixP & ddScattMatrixP::operator = (const ddScattMatrixP &rhs)
		{
			if (this == &rhs) return *this; // self-assignment check

			_Pnn = boost::shared_ptr<matrixop>(new matrixop(*(rhs._Pnn)));

			return *this;
		}

		void ddScattMatrixP::setP(const matrixop &val)
		{
			_Pnn = boost::shared_ptr<matrixop>(new matrixop(val));
		}

		matrixop ddScattMatrixP::getP() const
		{
			matrixop res(*(_Pnn.get()));
			return res;
		}

		matrixop ddScattMatrixF::mueller() const
		{
			_calcP();
			return *_Pnn;
		}

		matrixop ddScattMatrixF::getS() const
		{
			matrixop res(2,2,4);
			res.set(_sRe->get(2,0,0),2,0,0);
			res.set(_sIm->get(2,0,0),2,0,1);
			res.set(_sRe->get(2,0,1),2,0,2);
			res.set(_sIm->get(2,0,1),2,0,3);
			res.set(_sRe->get(2,1,0),2,1,0);
			res.set(_sIm->get(2,1,0),2,1,1);
			res.set(_sRe->get(2,1,1),2,1,2);
			res.set(_sIm->get(2,1,1),2,1,3);

			return res;
		}

		matrixop ddScattMatrixF::getF() const
		{
			matrixop res(2,2,4);
			res.set(_fRe->get(2,0,0),2,0,0);
			res.set(_fIm->get(2,0,0),2,0,1);
			res.set(_fRe->get(2,0,1),2,0,2);
			res.set(_fIm->get(2,0,1),2,0,3);
			res.set(_fRe->get(2,1,0),2,1,0);
			res.set(_fIm->get(2,1,0),2,1,1);
			res.set(_fRe->get(2,1,1),2,1,2);
			res.set(_fIm->get(2,1,1),2,1,3);

			return res;
		}

		void ddScattMatrixF::_calcS() const
		{
			using namespace std;
			const double PI = boost::math::constants::pi<double>();
			complex<double> i(0,1);
			complex<double> e01x(0,0), e01y(1,0), e01z(0,0), e02x(0,0), e02y(0,0), e02z(1,0);
			complex<double> a = conj(e01y), b=conj(e01z), c=conj(e02y), d=conj(e02z);
			complex<double> _Sn[4];
			complex<double> _fs[2][2];
			complexify(*_fRe,*_fIm,&_fs[0][0]);
			rtmath::phaseFuncs::convertFtoS(_fs,_Sn,_phi,a,b,c,d);

			_sRe->set(_Sn[0].real(),2,0,0);
			_sIm->set(_Sn[0].imag(),2,0,0);
			_sRe->set(_Sn[1].real(),2,0,1);
			_sIm->set(_Sn[1].imag(),2,0,1);
			_sRe->set(_Sn[2].real(),2,1,0);
			_sIm->set(_Sn[2].imag(),2,1,0);
			_sRe->set(_Sn[3].real(),2,1,1);
			_sIm->set(_Sn[3].imag(),2,1,1);
		}

		void ddScattMatrixF::_calcP() const
		{
			// Generates Snn and, by extension, Knn and Pnn
			using namespace std;
			complex<double> _S[4];
			complexify(*_sRe,*_sIm,&_S[0]);
			
			double Snn[4][4];
			// TODO: obey user mueller def selection
			rtmath::phaseFuncs::muellerBH(_S,Snn);
			_Pnn->fromDoubleArray(&Snn[0][0]);
		}

		void ddScattMatrixF::setF(const std::complex<double> fs[2][2])
		{
			_fRe->set(fs[0][0].real(),2,0,0);
			_fIm->set(fs[0][0].imag(),2,0,0);

			_fRe->set(fs[0][1].real(),2,0,1);
			_fIm->set(fs[0][1].imag(),2,0,1);

			_fRe->set(fs[1][0].real(),2,1,0);
			_fIm->set(fs[1][0].imag(),2,1,0);

			_fRe->set(fs[1][1].real(),2,1,1);
			_fIm->set(fs[1][1].imag(),2,1,1);

			_calcS();
			_calcP();

			_pol = sqrt( (pow(_Pnn->get(2,1,0),2) + pow(_Pnn->get(2,2,0),2) + pow(_Pnn->get(2,3,0),2))
				/ pow(_Pnn->get(2,0,0),2) );
		}
		
		/* Include this at a higher level
		void ddScattMatrixF::setF(std::istream &lss)
		{
			// This function reads directly from a string and extracts the appropriate values.
			// It compartamentalizes ddscat .fml reads into the appropriate classes.
			// May be called from public function or by constructor.
			// Called by istream operator.
			using namespace std; 
			double re, im;
			//istringstream lss(lin);
			lss >> _theta >> _phi;
			for (size_t i=0;i<4;i++)
			{
				lss >> re >> im;
				complex<double> nval(re,im);
				size_t j=i%2;
				size_t k=i/2;
				_vals[j][k] = nval;
			}
			
			_genS();
		}
		
		
		void ddScattMatrix::_genS()
		{
			// Generates Snn and, by extension, Knn and Pnn
			// TODO: verify Snn, Pnn and Knn
			using namespace std;
			const double PI = boost::math::constants::pi<double>();

			//complex<double> S[4];
			complex<double> i(0,1);

			complex<double> e01x(0,0), e01y(1,0), e01z(0,0), e02x(0,0), e02y(0,0), e02z(1,0);
			complex<double> a = conj(e01y), b=conj(e01z), c=conj(e02y), d=conj(e02z);

			//double cp = cos(2.0*PI*phi()/180.0);
			double cp = cos(_phi * PI / 180.0);
			//double sp = sin(2.0*PI*phi()/180.0);
			double sp = sin(_phi * PI / 180.0);
			_S[0] = -i * ( _vals[1][0] * (b * cp - a * sp) + _vals[1][1] * (d * cp - c * sp) );
			_S[1] = -i * ( _vals[0][0] * (a*cp + b * sp) + _vals[0][1] * (c * cp + d * sp) );
			_S[2] = i * ( _vals[0][0] * (b * cp - a * sp) + _vals[0][1] * (d * cp - c * sp) );
			_S[3] = i * ( _vals[1][0] * (a*cp + b * sp) + _vals[1][1] * (c*cp + d * sp) );
			
			rtmath::scattMatrix::_genExtinctionMatrix(_Knn, _S, _freq);
			rtmath::scattMatrix::_genMuellerMatrix(_Pnn,_S);
		}
		*/
	} // end ddscat
} // end rtmath

