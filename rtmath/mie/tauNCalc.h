#pragma once
#include "piNCalc.h"

namespace mie {

	class tauNCalc {
	public:
		tauNCalc(double theta);
		~tauNCalc();
		double calc(unsigned int n);
		double theta;
		double mu;
		piNCalc* pin;
	};


}; // end mie

