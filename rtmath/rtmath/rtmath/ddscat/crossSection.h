#pragma once
#include <string>
#include <vector>
#include <set>
#include <boost/shared_ptr.hpp>
#pragma warning(push)
#pragma warning( disable : 4521 )
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#pragma warning(pop)
#include "../matrixop.h"

/* This contains the necessary functions for figuring out the cross-sectional area of 
 * a 3d image. It works by first projecting the image along the desired plane, and then 
 * finding a representative concave hull. The hull is concave to better match small 
 * features on the projected surface. Then, the concave hull area and perimeter are 
 * determined */

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
		class crossSection
		{
		public:
			crossSection();
			crossSection(const std::vector<matrixop> &backend);
			crossSection(const pcl::PointCloud<pcl::PointXYZ> &backend);
			virtual ~crossSection();
			void writeVTKhull(const std::string &filename) const;
			double area() const;
			double perimeter() const;
			void construct(double alpha, double x, double y, double z);

			pcl::PointCloud<pcl::PointXYZ> _points, _projpoints;
			std::vector< pcl::Vertices > _polygons;
			mutable pcl::PointCloud<pcl::PointXYZ> _hullPts;
			double _area, _perimeter;
		};
	}
}

