#include "miePhaseFunc.h"
#include <cmath>
#include <complex>
namespace mie {

miePhaseFunc::miePhaseFunc(void)
{
}


miePhaseFunc::~miePhaseFunc(void)
{
}


//TODO: rewrite this function, giving dependency on rtmath-base provided matrix operations!
// See Liou page 261 for the correct form of the phase fonction based on the scattering 
// amplitude functions
void miePhaseFunc::calc(double mu, double mun, double phi, double pn, double *res[4][4])
{
	// This is the class-based array for calculating the phase function
	double phiangle=phi-pn;
	double muangle = mu-mun;
	// Spherical particles have S3=S4=0, so only have S1,S2(theta)
	// S are amplitude functions, p is the phase function
	// zero res
	for (unsigned int i=0; i<4; i++)
	{
		for (unsigned int j=0; j<4; j++)
			res[i][j] = 0;
	}

}
s
}; // end mie

