#pragma once
#include "../defs.h"
#include <string>
#include <vector>
#include <set>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include "../registry.h"

/* This contains the necessary functions for computing convex and concave hulls, 
 * both for writeout and for shapefile determinations */

namespace rtmath
{
	namespace ddscat
	{
		class hull;
		class hull_IO_output_registry {};
		class hull_provider_registry {};
		class convexHull;

		template <class hullType>
		struct DLEXPORT_rtmath_voronoi hull_provider
		{
			typedef std::function<
				boost::shared_ptr<hullType>
				(boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > )
				> hullGenerator;
			hullGenerator generator;

			const char* name;
		};
	}
	namespace registry
	{
		//extern template class usesDLLregistry<
		//	::rtmath::ddscat::hull_IO_output_registry,
		//	IO_class_registry_writer<::rtmath::ddscat::hull> >;

		extern template class usesDLLregistry<
			::rtmath::ddscat::hull_provider_registry,
			::rtmath::ddscat::hull_provider<::rtmath::ddscat::convexHull> >;

	}
	namespace ddscat
	{
		// This requires vtk
		// Move to a plugin
		//void writeVTKpolys(const std::string &filename, 
		//	const vtkSmartPointer< vtkPolyData > &src);

		//class hullData;

		class DLEXPORT_rtmath_voronoi hull
		{
		protected:
			hull(boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > backend);
			hull();
			//boost::shared_ptr<hullData> _p;
		public:
			/// Encapsulated in a static function to allow multiple hull generators to return cast objects to the 
			/// appropriate backends.
			//static boost::shared_ptr<hull> generate
			//	(boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > backend);

			virtual ~hull() {}
			/// Write raw input points and polygons
			//void writeVTKraw(const std::string &filename) const;
			/// Write hull points and polygons
			//void writeVTKhull(const std::string &filename) const;
			/// Hull volume
			virtual double volume() const = 0;
			/// Hull surface area
			virtual double surfaceArea() const = 0;
			/// Max distance between two points in hull
			virtual double maxDiameter() const = 0;
			/// Rotation coordinates to principle axes of hull
			virtual void principalAxes(double &beta, double &theta, double &phi) const = 0;

			virtual void area2d(double out[3]) const = 0;
			virtual void perimeter2d(double out[3]) const = 0;
		};

		/// \todo Move implementation code to vtk plugin
		class DLEXPORT_rtmath_voronoi convexHull : virtual public hull,
			virtual public ::rtmath::registry::usesDLLregistry<
			hull_provider_registry, hull_provider<convexHull> >
		{
		protected:
			convexHull(boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> >);
		public:
			/// Encapsulated in a static function to allow multiple hull generators to return cast objects to the 
			/// appropriate backends.
			static boost::shared_ptr<convexHull> generate
				(boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > backend);

			virtual ~convexHull() {}
			/// Construct the convex hull, and populate the quantities
			/// \todo Invoke in constructor?
			virtual void constructHull() = 0;
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

