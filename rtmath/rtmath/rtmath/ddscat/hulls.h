#pragma once
#include <string>
#include <vector>
#include <set>
#include <boost/shared_ptr.hpp>

#include "../matrixop.h"

//#include "shapefile.h"

/* This contains the necessary functions for computing convex and concave hulls, 
 * both for writeout and for shapefile determinations */

namespace pcl
{
	struct Vertices;
}

namespace rtmath
{
	class matrixop;

	namespace Garrett {
		class pointContainer;
	};

	namespace ddscat
	{

		void writeVTKpoints(const std::string &filename, const std::vector<matrixop> &src);

		class hull
		{
		public:
			hull(const std::vector<matrixop> &backend);
			virtual ~hull();
			double searchRadius, Mu, minAngle, maxAngle, maxSurfAngle;
			size_t maxNearestNeighbors;
			bool normalConsistency;
			void writeVTKhull(const std::string &filename) const;
			double volume() const;
			double surface_area() const;
		public:
			hull();
			std::vector<matrixop> _points;
			std::vector< pcl::Vertices > _polygons;
			mutable std::vector<matrixop> _hullPts;
			double _volume, _surfarea;
			size_t _nFaces;
		};

		class convexHull : public hull
		{
		public:
			convexHull(const std::vector<matrixop>&);
			virtual ~convexHull();
			void constructHull();
			double maxDiameter() const;
		};

		class concaveHull : public hull
		{
		public:
			concaveHull(const std::vector<matrixop>&);
			virtual ~concaveHull();
			void constructHull(double alpha);
		private:
			void _findVS();
		};
	}
}

