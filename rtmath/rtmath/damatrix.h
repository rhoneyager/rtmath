#pragma once
#include "../rtmath-base/matrixop.h"
#include "../rtmath-base/quadrature.h"

namespace rtmath {

	class damatrixfunc
	{
	public:
		double eval(unsigned int i, unsigned int j, 
			double tau, double mu, double mun, double phi, double phin);
		// eval is 
	};

	class daevalmuminor : public rtmath::evalfunction
	{
	public:
		// Here, val is mu'
		virtual double eval(double val) const;
		virtual double operator() (double) const;
	};

	class daevalmu : public rtmath::evalfunction
	{
	public:
		void setvals(double phi, double phin, double php, double mu, double mun);
		virtual double eval(double val) const;
		virtual double operator() (double) const;
	private:
		double _phi;
		double _phin;
		double _php;
		double _mu;
		double _mun;
		daevalmuminor targets;
	};

	class daevalphi : public rtmath::evalfunction
	{
	public:
		void setvals(double phi, double phin, double mu, double mun);
		virtual double eval(double val) const;
		virtual double operator() (double) const;
	private:
		double _phi;
		double _phin;
		double _mu;
		double _mun;
		daevalmu mueval;
	};

class damatrix :
	public matrixop
	// damatrix is derived from matrixop
	// It implements the standard matrix algebra, except multiplication
	// A product implies matrix mult. and integration over adjoining angles
{
public:
	damatrix(const std::vector<unsigned int> &size) : matrixop(size)
	{
		_mu = 0;
		_mun = 0;
		_ppn = 0;
	}
	virtual ~damatrix(void) {}
	damatrix (const damatrix & rhs) : matrixop(rhs) {}; // copy constructor
	virtual damatrix operator * (const damatrix&) const;
	void setparams(double mu, double mun, double ppn);
	//matrixop& eval(); // eval at setparams
protected:
	double _mu;
	double _mun;
	double _ppn;
};

}; // end rtmath
