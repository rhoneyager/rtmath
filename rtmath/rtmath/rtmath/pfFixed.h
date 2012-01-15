#pragma once

/* pfFixed - a class which reads a precomputed phase function from a file, such as those used by Evans' DA algorithms.
 * The resultant phase function may be evaluated in the usual manner. TODO: add a saving routine to daPf so that previous 
 * results may be loaded again without calculation.
 * Since the loaded phase function cannot cover all possible points in the domain, linear interpolation will be used.
 * TODO: add in spherical-harmonic methods also for more accurate calculation.
 */

#include "da/daPf.h"
#include "matrixop.h"
#include <string>
#include <memory>

namespace rtmath {

	

}; // end namespace rtmath

