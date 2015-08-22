#pragma once
#include "defs.h"
#include <complex>
#include <functional>
#include <string>
#include <sstream>
#include <vector>
#include <boost/parameter/keyword.hpp>
#include <boost/parameter/name.hpp>
#include <boost/parameter/preprocessor.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/math/constants/constants.hpp>
#include "convertLength.h"
#include "../units.h"
#include "../zeros.h"
#include "unitOptions.h"
#include <Ryan_Debug/error.h>
#include <Ryan_Debug/logging_base.h>

namespace rtmath {
	namespace units {

		/** \brief Function to convert between length and volume-based measurements.
		 *
		 * You provide either a length, with units and type, or a volume.
		 * The conversion respects aspect ratio and volume fraction.
		 **/
		BOOST_PARAMETER_FUNCTION( (double),
			convertVolume,
			tag,
			(required
				(in_length_value, (double))
				(in_length_type, (std::string))
				(out_length_type, (std::string))
			)
			(optional
				(ar, *, 1)
				(vf, *, 1)
				(in_length_units, *, std::string("um"))
				(out_length_units, *, std::string("um"))
			) )
		{
		}
	}
}

