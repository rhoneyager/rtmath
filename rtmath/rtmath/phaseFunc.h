#pragma once

#include<complex>
#include "enums.h"
#include "matrixop.h"

namespace rtmath {

	class phaseFunc
	{
public:
	// This fuction is designed to be pure virtual, so that mie and other
	// phase functions can be included in atmospheric layers

	// TODO: rewrite to remove dependency on x and m - these are dependent on the scattering scheme!
	phaseFunc(void);
	virtual ~phaseFunc(void);
	// eval evaluates P for reduced mur and phir (mu-mu0, phi-phi0) to get proper directional data
	//virtual void eval(double mu, double mun, double phir, double Pnn[4][4], double res[4][4]);
	// calc constructs the base P matrix, centered on mu=mu0
	virtual void calc(double mu, std::complex<double> &m, double x, double Pnn[4][4]) = 0;
	// And, give the same function as a matrixop class
	virtual void calc(double mu, std::complex<double> &m, double x, matrixop &Pnn);
	const std::vector<unsigned int> size()
	{
		std::vector<unsigned int> res;
		res.push_back(4);
		res.push_back(4);
		return res;
	}
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

class phaseFuncRotator
{
public:
	// Take a reference to the actual target
	phaseFuncRotator(phaseFunc &target, std::complex<double> &m, double x);
	// For most phasefuncs, calc(mu) is calc(alpha), refed to 
	// the total angle for single scattering.
	// We need more.
	void rotate(rtselec::rtselec RT, double mu, double mun, double alpha, double phi, double phin,
		matrixop &res);
	void rotate(rtselec::rtselec RT, double mu, double mun, double alpha, double phi, double phin,
		double res[4][4]);
	static void rotate(rtselec::rtselec RT, const matrixop &Pa, 
		const mapid &varmap, matrixop &res, double alpha);
private:
	std::complex<double> m;
	double x;
	phaseFunc *target;

};

}; // end namespace rtmath
