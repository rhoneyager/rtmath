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
}; // end namespace rtmath

