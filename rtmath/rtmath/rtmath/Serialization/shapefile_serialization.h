#pragma once
#pragma deprecated(shapefile_serialization_h)

#include <boost/serialization/export.hpp>

namespace rtmath
{
	namespace ddscat
	{
		class shapefile;
	}
}

namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapefile & g, const unsigned int version);
	}
}

BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapefile)
