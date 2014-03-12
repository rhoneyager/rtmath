#pragma once
// Private header to share definitions of shapestats objects


namespace rtmath {
	namespace ddscat {
		namespace stats {
			extern boost::filesystem::path pHashShapes, pHashStats;
			extern bool autoHashShapes;
			extern bool autoHashStats;
			extern std::vector<boost::tuple<double, double, double> > defaultRots;
			extern bool doVoronoi;
			extern bool disableVoronoi;
		}
	}
}
