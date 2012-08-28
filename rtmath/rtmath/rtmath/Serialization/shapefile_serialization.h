#pragma once

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

