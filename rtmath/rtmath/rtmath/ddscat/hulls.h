#pragma once
#include <string>
#include <vector>
#include <set>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>

/* This contains the necessary functions for computing convex and concave hulls, 
 * both for writeout and for shapefile determinations */

namespace rtmath
{
	namespace ddscat
	{

		//void writeVTKpoints(const std::string &filename, const pcl::PointCloud<pcl::PointXYZ> &src);

		class hullData;

		class hull
		{
		public:
			hull(const Eigen::Matrix<float, Eigen::Dynamic, 3> &backend);
			virtual ~hull() {}
			void writeVTKhull(const std::string &filename) const;
			double volume() const;
			double surfaceArea() const;
		protected:
			boost::shared_ptr<hullData> _p;
		public:
			hull();
		};

		class convexHull : public hull
		{
		public:
			convexHull(const Eigen::Matrix<float, Eigen::Dynamic, 3>&);
			virtual ~convexHull() {}
			void constructHull();
			double maxDiameter() const;
			double maxDiameter(Eigen::Matrix<float, 2, 3> &range) const;
		};

		/*
		class concaveHull : public hull
		{
		public:
			concaveHull(const Eigen::Matrix<float, Eigen::Dynamic, 3>&);
			virtual ~concaveHull() {}
			void constructHull(double alpha);
		};
		*/
	}
}

