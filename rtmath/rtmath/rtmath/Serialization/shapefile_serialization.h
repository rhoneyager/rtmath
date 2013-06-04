#pragma once
#pragma message("TODO: no need for separate serialization header if forward declared properly")

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
