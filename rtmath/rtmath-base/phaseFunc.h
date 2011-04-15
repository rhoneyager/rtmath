#pragma once

#include<complex>

namespace rtmath {

class phaseFunc
{
public:
	// This fuction is designed to be pure virtual, so that mie and other
	// phase functions can be included in atmospheric layers
	phaseFunc(void);
	virtual ~phaseFunc(void);
	// eval evaluates P for reduced mur and phir (mu-mu0, phi-phi0) to get proper directional data
	virtual void eval(double mu, double mun, double phir, double Pnn[4][4], double res[4][4]);
	// calc constructs the base P matrix, centered on mu=mu0
	virtual void calc(double mu, std::complex<double> &m, double x, double Pnn[4][4]) = 0;
};

class scattMatrix
{
public:
	scattMatrix(void);
	virtual ~scattMatrix(void);
	// Calculate the scattering amplitude matrix for a given mu
	virtual void calc(double mu, double Snn[4][4], std::complex<double> Sn[4]) = 0;
protected:
	void _genMuellerMatrix(double Snn[4][4], std::complex<double> Sn[4]);
};

}; // end namespace rtmath
