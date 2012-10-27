#pragma once
#include <string>
#include <vector>
#include <set>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#pragma warning(push)
#pragma warning( disable : 4521 )
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#pragma warning(pop)

/* This contains the necessary functions for computing convex and concave hulls, 
 * both for writeout and for shapefile determinations */

namespace pcl
{
	struct Vertices;
}

namespace rtmath
{
	namespace Garrett {
		class pointContainer;
	};

	namespace ddscat
	{

		void writeVTKpoints(const std::string &filename, const pcl::PointCloud<pcl::PointXYZ> &src);
		//void writeVTKpoints(const std::string &filename, const std::vector<matrixop> &src);
		void writeVTKpoints(const std::string &filename, const std::vector<Eigen::Vector3f> &src);

		class hull
		{
		public:
			hull(const std::vector<Eigen::Vector3f> &backend);
			//hull(const std::vector<matrixop> &backend);
			hull(const pcl::PointCloud<pcl::PointXYZ>::Ptr &backend);
			virtual ~hull();
			void writeVTKhull(const std::string &filename) const;
			double volume() const;
			double surface_area() const;
			bool hull_enabled;
		public:
			hull();
			pcl::PointCloud<pcl::PointXYZ>::Ptr _points;
			std::vector< pcl::Vertices > _polygons;
			mutable pcl::PointCloud<pcl::PointXYZ>::Ptr _hullPts;
			double _volume, _surfarea;
		};

		class convexHull : public hull
		{
		public:
			convexHull(const std::vector<Eigen::Vector3f> &);
			//convexHull(const std::vector<matrixop>&);
			convexHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr &);
			virtual ~convexHull();
			void constructHull();
			double maxDiameter() const;
		};

		class concaveHull : public hull
		{
		public:
			concaveHull(const std::vector<Eigen::Vector3f> &);
			//concaveHull(const std::vector<matrixop>&);
			concaveHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr &);
			virtual ~concaveHull();
			void constructHull(double alpha);
		private:
			void _findVS();
		};
	}
}

