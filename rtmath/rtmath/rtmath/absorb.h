#pragma once

/* absorb_basic - contains the functions for calculating atmospheric absorption 
 * due to water vapor, oxygen and nitrogen. This is an adaptation of Dr. Liu's 
 * code (in mwrt3/sub0/absorb.f), which considers absorption in a bulk atmosphere.
 * Since my code needs the absorption in a small layer, his code requires some 
 * adaptation.
 */

#include <set>
#include <map>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include "error/debug.h"

namespace rtmath {
	namespace absorb {

		// The absorb class is a base class to handle different
		// methods of calculating absorption. By having a 
		// base class, sets of absorption calculations can be 
		// implemented, providing the ability, for example, for 
		// water absorption to be turned off while o3 is turned 
		// on. This is meant to be an alternative to lbl, and 
		// will eventually handle things like band models, too.
		class absorb {
			public:
				absorb() {}
				virtual ~absorb();
				// wvnum is wavenumbers, T in kelvin, p in hPa
				virtual double tau(double T, double p, double wvnum) const = 0;
				// virtual double tau(double quant, dim)
				// Provide easily accessable function for 
				// transmittance calculation:
				static double T(double tau);
			protected:
				double _wvtofreq(double wvnum);
		};


		// Might as well set some functions that handle absorption
		// b.n.: These functions are member functions because they 
		// have virtual class inheritance. However, though they are 
		// member functions, their classes have no member data. The
		// initialization is inline, so it doesn't matter too much.

		class collide : public absorb {
			public:
				inline collide() {}
				virtual ~collide() {}
				virtual double tau(double T, double p, double wvnum) const;
		};


	}; // end namespace absorb
}; // end namespace rtmath

