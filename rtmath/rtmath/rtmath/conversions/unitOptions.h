#pragma once
#include "../defs.h"
#include <boost/parameter/keyword.hpp>
#include <boost/parameter/name.hpp>
#include <boost/parameter/preprocessor.hpp>

namespace rtmath {
	namespace units {
		namespace keywords {
			BOOST_PARAMETER_NAME(frequency)
			BOOST_PARAMETER_NAME(temperature)
			BOOST_PARAMETER_NAME(salinity)
			BOOST_PARAMETER_NAME(temp_units)
			BOOST_PARAMETER_NAME(freq_units)
			BOOST_PARAMETER_NAME(salinity_units)
			BOOST_PARAMETER_NAME(m)
			BOOST_PARAMETER_NAME(volume_fraction)
			BOOST_PARAMETER_NAME(provider)
			BOOST_PARAMETER_NAME(in_length_value)
			BOOST_PARAMETER_NAME(max_dimension)
			BOOST_PARAMETER_NAME(mean_volume_diameter)
			BOOST_PARAMETER_NAME(mean_volume_radius)
			BOOST_PARAMETER_NAME(in_length_units)
			BOOST_PARAMETER_NAME(in_length_type)
			BOOST_PARAMETER_NAME(out_length_type)
			BOOST_PARAMETER_NAME(out_length_units)
			BOOST_PARAMETER_NAME(ar)
			BOOST_PARAMETER_NAME(in_density)
			BOOST_PARAMETER_NAME(out_density)
			BOOST_PARAMETER_NAME(in_volume)
			BOOST_PARAMETER_NAME(in_aeff)
			BOOST_PARAMETER_NAME(in_substance)
			BOOST_PARAMETER_NAME(out_substance)
			BOOST_PARAMETER_NAME(in_volume_fraction)
			BOOST_PARAMETER_NAME(out_volume_fraction)
		}
	}
}

