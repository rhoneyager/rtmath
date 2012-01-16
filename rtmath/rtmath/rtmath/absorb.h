#pragma once

/* absorb - contains the functions for calculating atmospheric absorption 
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
#include "atmos.h"

namespace rtmath {
	namespace atmos {

		// Might as well set some functions that handle absorption
		// b.n.: These functions are member functions because they 
		// have virtual class inheritance. However, though they are 
		// member functions, their classes have no member data. The
		// initialization is inline, so it doesn't matter too much.


		// Collision-induced absorption by Pardo et al. (2000)
		// Pardo, J. R., E. Serabyn, and J. Cernicharo, 
		//  Submillimeter atmospheric transmission measurements
		// on Mauna Kea during extremely dry El Nino
		// consitions: Implications for broadband opacity 
		// contributions. J.Q.S.R.T., 67, 169-180, 2000.
		// 1.29 times of N2-N2 collision absorption to account
		// for N2-O2 and O2-O2 collisions
		class collide : public absorber {
			public: virtual double deltaTau(double nu) const;
		};

		// N2 collision absorption
		// P. Rosenkranz (1998)
		class abs_N2 : public absorber {
			public: virtual double deltaTau(double nu) const;
		};

		// H2O absorption
		// Rosenkranz (1998)
		class abs_H2O : public absorber {
			public: virtual double deltaTau(double nu) const;
		};

		// O2 absorption
		// Rosenkranz (1995)
		class abs_O2 : public absorber {
			public: virtual double deltaTau(double nu) const;
		};

	}; // end namespace absorb
}; // end namespace rtmath

