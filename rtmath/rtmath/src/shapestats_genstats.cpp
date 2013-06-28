#include "../rtmath/Stdafx.h"

#include <Ryan_Serialization/serialization.h>
#include "../rtmath/matrixop.h"
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/ddscat/shapestats.h"

#include "../rtmath/Serialization/shapestats_serialization.h"

namespace rtmath {
	namespace ddscat {
		boost::shared_ptr<shapeFileStats> shapeFileStats::genStats(
			const std::string &shpfile, const std::string &statsfile)
		{
			// Generate basic stats for a file.
			rtmath::ddscat::shapefile shp(shpfile);
			rtmath::ddscat::shapeFileStats sstats(shp);
			sstats.calcStatsBase();
			sstats.calcStatsRot(0,0,0);

			if (statsfile.size())
			{
				::Ryan_Serialization::write<rtmath::ddscat::shapeFileStats >(sstats,statsfile);
			}

			boost::shared_ptr<shapeFileStats> p(new shapeFileStats(sstats));
			return p;
		}

	}
}

