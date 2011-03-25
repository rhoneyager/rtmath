#pragma once
#include "piNCalc.h"

namespace mie {

	class tauNCalc {
	public:
		tauNCalc(double mu);
		~tauNCalc();
		double calc(unsigned int n);
		double mu;
		piNCalc* pin;
	};


}; // end mie

