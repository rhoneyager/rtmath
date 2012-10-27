#include "../rtmath/Stdafx.h"
#pragma warning( disable : 4996 )
#pragma warning( disable : 4305 )
#pragma warning( disable : 4244 )

#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>

// Need to disable PCL warnings that are extraneous
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/PolygonMesh.h>

//#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/ddscat/hulls.h"
#include "../rtmath/error/error.h"
#include "../rtmath/PCLlink.h"

#pragma GCC diagnostic pop

namespace rtmath
{
	namespace ddscat
	{
		void writeVTKpoints(const std::string &filename, const pcl::PointCloud<pcl::PointXYZ> &src)
		{
			sensor_msgs::PointCloud2 pbc;
			pcl::toROSMsg(src, pbc);
			pcl::io::saveVTKFile(filename.c_str(),pbc);
		}

		void writeVTKpoints(const std::string &filename, const std::vector<Eigen::Vector3f> &src)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
			const size_t N = src.size();
			cloud->reserve(N);
			
			for (auto it = src.begin(); it != src.end(); it++)
			{
				const double x = (*it)(0);
				const double y = (*it)(1);
				const double z = (*it)(2);
				cloud->push_back(pcl::PointXYZ(x,y,z));
			}

			sensor_msgs::PointCloud2 pbc;
			pcl::toROSMsg(*cloud, pbc);
			pcl::io::saveVTKFile(filename.c_str(),pbc);
		}
		/*
		void writeVTKpoints(const std::string &filename, const std::vector<matrixop> &src)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
			const size_t N = src.size();
			cloud->reserve(N);
			
			for (auto it = src.begin(); it != src.end(); it++)
			{
				const double x = it->get(2,0,0);
				const double y = it->get(2,0,1);
				const double z = it->get(2,0,2);
				cloud->push_back(pcl::PointXYZ(x,y,z));
			}

			sensor_msgs::PointCloud2 pbc;
			pcl::toROSMsg(*cloud, pbc);
			pcl::io::saveVTKFile(filename.c_str(),pbc);
		}
		*/
		hull::hull()
		{
			_volume = 0;
			_surfarea = 0;
			hull_enabled = true;
			_points = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
			_hullPts = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
		}

		hull::~hull() { }

		hull::hull(const std::vector<Eigen::Vector3f> &backend)
		{
			_points->reserve(backend.size());
			for (auto it = backend.begin(); it != backend.end(); ++it)
			{
				const double x = (*it)(0);
				const double y = (*it)(1);
				const double z = (*it)(2);
				_points->push_back(pcl::PointXYZ(x,y,z));
			}

			_hullPts = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
			_volume = 0;
			_surfarea = 0;
		}

		hull::hull(const pcl::PointCloud<pcl::PointXYZ> &backend)
		{
			_points = backend;
			_hullPts = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
			_volume = 0;
			_surfarea = 0;
		}

		void hull::writeVTKhull(const std::string &filename) const
		{
			pcl::PolygonMesh triangles;
			triangles.polygons = _polygons;
			sensor_msgs::PointCloud2 pbc;
			pcl::toROSMsg(_hullPts, pbc);
			triangles.cloud = pbc;
			
			//generateMeshGreedy(*this,cloud,triangles);
			pcl::io::saveVTKFile (filename.c_str(), triangles);
		}

		convexHull::convexHull(const std::vector<Eigen::Vector3f> &src)
			: hull(src)
		{
		}

		convexHull::convexHull(const pcl::PointCloud<pcl::PointXYZ> &src)
			: hull(src)
		{
		}

		convexHull::~convexHull()
		{
		}

		void convexHull::constructHull()
		{
			_hullPts->clear();

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(_points);

			pcl::ConvexHull<pcl::PointXYZ> chull;
			// Dimension is not set because it is also used for 2d shapes
			//chull.setDimension(3); // Dimension 3 fails for dendrites
			chull.setComputeAreaVolume(true);
			chull.setInputCloud (cloud);
			chull.reconstruct (_hullPts.get(), _polygons);
			_volume = chull.getTotalVolume();
			_surfarea = chull.getTotalArea();
		}

		double convexHull::maxDiameter() const
		{
			double maxD = 0;
			const pcl::PointCloud<pcl::PointXYZ>::Ptr base;
			if (hull_enabled) base = _hullPts;
			else base = _points;

			// This function is too slow
			//pcl::PointXYZ min, max;
			//maxD = pcl::getMaxSegment(*base, min, max);

			for (auto it = base->begin(); it != base->end(); ++it)
			{
				for (auto ot = base->begin(); ot != base->end(); ++ot)
				{
					double d = (it->x - ot->x) * (it->x - ot->x);
					d += (it->y - ot->y,2.0f) * (it->y - ot->y,2.0f);
					d += (it->z - ot->z,2.0f) * (it->z - ot->z,2.0f);
					if (d > maxD) maxD = d;
				}
			}
			return sqrt(maxD);
		}

		double hull::volume() const
		{
			return _volume;
		}

		double hull::surface_area() const
		{
			return _surfarea;
		}

		concaveHull::concaveHull(const std::vector<Eigen::Vector3f> &src)
			: hull(src)
		{
		}

		concaveHull::concaveHull(const pcl::PointCloud<pcl::PointXYZ> &src)
			: hull(src)
		{
		}

		concaveHull::~concaveHull()
		{
		}
		
		void concaveHull::constructHull(double alpha)
		{
			_hullPts->clear();

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (_points);
			//std::copy(_points.begin(),_points.end(),cloud->begin());

			pcl::ConcaveHull<pcl::PointXYZ> chull;
			//chull.setDimension(3);
			chull.setInputCloud (cloud);
			chull.setAlpha(alpha);
			chull.reconstruct (_hullPts.get(), _polygons);

			//_findVS();
		}

		void concaveHull::_findVS()
		{
			throw rtmath::debug::xUnimplementedFunction();
		}

	}
}

