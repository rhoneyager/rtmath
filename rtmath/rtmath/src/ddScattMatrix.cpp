#include "Stdafx-ddscat.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <complex>
#include <boost/math/constants/constants.hpp>
#include "../rtmath/ddscat/ddScattMatrix.h"
#include "../rtmath/units.h"
#include "../rtmath/phaseFunc.h"
//#include "../rtmath/coords.h"
#include "../rtmath/error/error.h"

namespace rtmath {
	namespace ddscat {

		ddScattMatrix::ddScattMatrix(double freq, double theta, double phi, double thetan, double phin) :
			_pol(0),
			_freq(freq),
			_theta(theta),
			_thetan(thetan),
			_phi(phi),
			_phin(phin)
		{
		}

		ddScattMatrix::~ddScattMatrix() {}

		bool ddScattMatrix::operator<(const ddScattMatrix &rhs) const
		{
#define comp(x) if (x != rhs.x) return (x < rhs.x);
			comp(_freq);
			comp(_phi);
			comp(_theta);
			comp(_phin);
			comp(_thetan);
			comp(_pol);
#undef comp
			return false;
		}

		bool ddScattMatrix::compareTolHeader(const ddScattMatrix &rhs, double tol) const
		{
			// NOTE: the preprocessor handles math in a different order!
#define tol(x) if ( abs( 100.*x/rhs.x - 100.0 ) > tol) return false;
			tol(_freq);
			tol(_theta);
			tol(_thetan);
			tol(_phi);
			tol(_phin);
			tol(_pol);
#undef tol
			return true;
		}

		ddScattMatrix::PnnType ddScattMatrix::mueller() const
		{
			return _Pnn;
		}

		void ddScattMatrix::_calcPol()
		{
			_pol = sqrt( (pow(_Pnn(1,0),2.) + 
				pow(_Pnn(2,0),2.) + 
				pow(_Pnn(3,0),2.))
				/ pow(_Pnn(0,0),2.) );
		}

		ddScattMatrixF::~ddScattMatrixF() {}

		ddScattMatrixF::PnnType ddScattMatrixF::mueller() const
		{
			_calcP();
			return _Pnn;
		}

		void ddScattMatrixF::_calcS()
		{
			using namespace std;
			const double PI = boost::math::constants::pi<double>();
			complex<double> i(0,1);
			complex<double> e01x(0,0), e01y(1,0), e01z(0,0), e02x(0,0), e02y(0,0), e02z(1,0);
			complex<double> a = conj(e01y), b=conj(e01z), c=conj(e02y), d=conj(e02z);
			rtmath::phaseFuncs::convertFtoS(_f,_s,_phi,a,b,c,d);
		}

		void ddScattMatrixF::_calcP() const
		{
			// Generates Snn and, by extension, Knn and Pnn
			using namespace std;
			rtmath::phaseFuncs::muellerBH(_s, _Pnn);
		}

		void ddScattMatrixF::setF(const FType& fs)
		{
			_f = fs;
			_calcS();
			_calcP();

			_calcPol();
		}

		ddScattMatrixF ddScattMatrixF::operator+(const ddScattMatrixF& rhs) const
		{
			if (!compareTolHeader(rhs)) throw(debug::xBadInput(
				"ddScattMatrices are not of the same header type"));
			ddScattMatrixF res;
			
			res._f = this->_f + rhs._f;
			res._s = this->_s + rhs._s;
			res.mueller();
			res._calcPol();

			return res;
		}

		ddScattMatrixF ddScattMatrixF::operator*(double rhs) const
		{
			ddScattMatrixF res = *this;
			
			res._f = this->_f * rhs;
			res._s = this->_s * rhs;
			res.mueller();
			res._calcPol();

			return res;
		}

		ddScattMatrixP ddScattMatrixP::operator+(const ddScattMatrixP& rhs) const
		{
			if (!compareTolHeader(rhs)) throw(debug::xBadInput(
				"ddScattMatrices are not of the same header type"));
			ddScattMatrixP res;
			
			res._Pnn = this->_Pnn + rhs._Pnn;
			res.mueller();
			res._calcPol();

			return res;
		}

		ddScattMatrixP ddScattMatrixP::operator*(double rhs) const
		{
			ddScattMatrixP res = *this;
			
			res._Pnn = this->_Pnn * rhs;
			res.mueller();
			res._calcPol();

			return res;
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
	}
}
