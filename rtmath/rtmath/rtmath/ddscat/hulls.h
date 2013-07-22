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
		// This requires vtk
		//void writeVTKpolys(const std::string &filename, 
		//	const vtkSmartPointer< vtkPolyData > &src);

		class hullData;

		class hull
		{
		public:
			hull(const Eigen::Matrix<float, Eigen::Dynamic, 3> &backend);
			virtual ~hull() {}
			/// Write raw input points and polygons
			void writeVTKraw(const std::string &filename) const;
			/// Write hull points and ploygons
			void writeVTKhull(const std::string &filename) const;
			/// Hull volume
			double volume() const;
			/// Hull surface area
			double surfaceArea() const;
			/// Max distance between two points in hull
			double maxDiameter() const;
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
			/// Construct the convex hull, and populate the quantities
			/// \todo Invoke in constructor
			void constructHull();
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

