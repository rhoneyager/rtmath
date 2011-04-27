#include "Stdafx.h"
#include "layer.h"
#include "../rtmath-base/phaseFunc.h"
#include "../rtmath-base/matrixop.h"
#include "../rtmath-base/quadrature.h"
#include "damatrix.h"
#include <map>
#include <vector>

namespace rtmath
{

dalayer::dalayer(phaseFunc *pf, double alb)
{
	_pf = pf;
	_alb = alb;
}


dalayer::~dalayer(void)
{
}


damatrix calcR(double tau, double phi, double phin, 
	double mu, double mun)
{
	throw;
	std::vector<unsigned int> siz;
	siz.push_back(4);
	siz.push_back(4);
	damatrix nul(siz);
	return nul;
	// Solution by recursion
	// Yeah, this stinks, but it's how DA is defined
	// Written so that recursive calls are not made
	//  so as not to kill the stack

}

damatrix calcT(double tau, double phi, double phin, 
	double mu, double mun)
{
	throw;
	std::vector<unsigned int> siz;
	siz.push_back(4);
	siz.push_back(4);
	damatrix nul(siz);
	return nul;
}

}; // end rtmath


