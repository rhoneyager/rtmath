#pragma once
#include "../rtmath-base/matrixop.h"
#include "../rtmath-base/quadrature.h"

namespace rtmath {

	class damatrix;

	class daevalmu : public rtmath::evalfunction
	{
	public:
		void setvals(double phi, double phin, double php, 
			double mu, double mun,
			unsigned int i, unsigned int j);
		virtual double eval(double val) const;
		virtual double operator() (double) const;
	private:
		double _phi;
		double _phin;
		double _php;
		double _mu;
		double _mun;
		unsigned int _i, _j;
		damatrix *targeta, *targetb;
	};

	class daevalphi : public rtmath::evalfunction
	{
	public:
		void setvals(double phi, double phin, 
			double mu, double mun,
			unsigned int i, unsigned int j);
		// TODO: recode quadrature to perform on matrices
		virtual double eval(double val) const;
		virtual double operator() (double) const;
		daevalmu *mueval;
	private:
		double _phi;
		double _phin;
		double _mu;
		double _mun;
		unsigned int _i, _j;
	};

class damatrix :
	public matrixop
	// damatrix is derived from matrixop
	// It implements the standard matrix algebra, except multiplication
	// A product implies matrix mult. and integration over adjoining angles
{
public:
	// TODO: add upconvert structure from matrixop
	damatrix(const std::vector<unsigned int> &size) : matrixop(size)
	{
	}
	virtual ~damatrix(void) {}
	damatrix (const damatrix & rhs) : matrixop(rhs) {}; // copy constructor
	virtual damatrix operator * (const damatrix&) const;
	//void setparams(double mu, double mun, double ppn);
	/* For eval, a,b,c may be some combination of 
	   mu, mu', mu0, phi, phi', phi0
	   */
	virtual double eval(unsigned int i, unsigned int j,
		double a, double b, double c); // eval at setparams
protected:
	double _phi, _phip, _phin, _mu, _mup, _mun;
};

}; // end rtmath
