#pragma once

#include "matrixop_serialization.h"

namespace rtmath
{
	namespace ddscat
	{
		class shapeFileStatsRotated;
	}
}

namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapeFileStatsRotated & g, const unsigned int version);
	}
}

