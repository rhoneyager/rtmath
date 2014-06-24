#pragma once
#include "../defs.h"
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
		// Move to a plugin
		//void writeVTKpolys(const std::string &filename, 
		//	const vtkSmartPointer< vtkPolyData > &src);

		class hullData;

		class DLEXPORT_rtmath_voronoi hull
		{
		public:
			hull(boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > backend);
			virtual ~hull() {}
			/// \brief Write raw input points and polygons
			/// \todo Move to plugin
			void writeVTKraw(const std::string &filename) const;
			/// \brief Write hull points and polygons
			/// \todo Move to plugin
			void writeVTKhull(const std::string &filename) const;
			/// Hull volume
			double volume() const;
			/// Hull surface area
			double surfaceArea() const;
			/// Max distance between two points in hull
			double maxDiameter() const;
			/// Rotation coordinates to principle axes of hull
			void principalAxes(double &beta, double &theta, double &phi) const;

			void area2d(double out[3]) const;
			void perimeter2d(double out[3]) const;
		protected:
			boost::shared_ptr<hullData> _p;
		public:
			hull();
		};

		/// \todo Move implementation code to vtk plugin
		class DLEXPORT_rtmath_voronoi convexHull : public hull
		{
		public:
			convexHull(boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > );
			virtual ~convexHull() {}
			/// Construct the convex hull, and populate the quantities
			/// \todo Invoke in constructor
			void constructHull();
		};

		/* // Disabled because the VTK hull algorithms are SLOW, and setting alpha requires all points!
		class DLEXPORT_rtmath_voronoi concaveHull : public hull
		{
		public:
			concaveHull(boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> >);
			virtual ~concaveHull() {}
			void constructHull(double alpha);
		};
		*/
	}
}

