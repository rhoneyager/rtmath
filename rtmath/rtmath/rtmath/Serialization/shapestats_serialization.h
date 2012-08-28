#pragma once

#include "matrixop_serialization.h"
#include "shapestatsRotated_serialization.h"
#include "shapefile_serialization.h"

namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapeFileStatsBase & g, const unsigned int version);

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapeFileStats & g, const unsigned int version);
	}
}
