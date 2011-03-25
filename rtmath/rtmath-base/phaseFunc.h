#pragma once

namespace rtmath {

class phaseFunc
{
public:
	// This fuction is designed to be pure virtual, so that mie and other
	// phase functions can be included in atmospheric layers
	phaseFunc(void);
	virtual ~phaseFunc(void);
	virtual void calc(double mu, double mun, double phi, double pn, double *res[4][4]) = 0;
};

}; // end namespace rtmath
