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
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/mls.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/PolygonMesh.h>

//#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/ddscat/crossSection.h"
#include "../rtmath/ddscat/hulls.h"
#include "../rtmath/error/error.h"
#include "../rtmath/PCLlink.h"

#pragma GCC diagnostic pop

namespace rtmath
{
	namespace ddscat
	{
		crossSection::crossSection()
		{
			_perimeter = 0;
			_area = 0;
		}

		crossSection::~crossSection()
		{
		}

		crossSection::crossSection(const std::vector<matrixop> &backend)
		{
			_points.reserve(backend.size());
			for (auto it = backend.begin(); it != backend.end(); ++it)
			{
				const double x = it->get(2,0,0);
				const double y = it->get(2,0,1);
				const double z = it->get(2,0,2);
				_points.push_back(pcl::PointXYZ(x,y,z));
			}

			_hullPts = _points;
			_perimeter = 0;
			_area = 0;
		}

		crossSection::crossSection(const pcl::PointCloud<pcl::PointXYZ> &backend)
		{
			_points = backend;
			_hullPts = backend;
			_perimeter = 0;
			_area = 0;
		}


		double crossSection::area() const
		{
			return _area;
		}

		double crossSection::perimeter() const
		{
			return _perimeter;
		}

		void crossSection::writeVTKhull(const std::string &filename) const
		{
			pcl::PolygonMesh triangles;
			triangles.polygons = _polygons;
			sensor_msgs::PointCloud2 pbc;
			pcl::toROSMsg(_hullPts, pbc);
			triangles.cloud = pbc;
			
			//generateMeshGreedy(*this,cloud,triangles);
			pcl::io::saveVTKFile (filename.c_str(), triangles);
		}

		void crossSection::construct(double alpha, double x, double y, double z)
		{
			// Project to axis

			// Create a set of planar coefficients with X=Y=0,Z=1
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
			coefficients->values.resize (4);
			coefficients->values[0] = x; // TODO: Check these
			coefficients->values[1] = y;
			coefficients->values[2] = z;
			coefficients->values[3] = 0;

			// Input cloud pointer of correct type is required.
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
			const size_t N = _points.size();
			cloud->resize(N);
			std::copy(_points.begin(),_points.end(),cloud->begin());

			// Create the filtering object
			pcl::ProjectInliers<pcl::PointXYZ> proj;
			proj.setModelType (pcl::SACMODEL_PLANE);
			proj.setInputCloud (cloud);
			proj.setModelCoefficients (coefficients);
			proj.filter (_projpoints);

			// Concave hull
			concaveHull cnc(_projpoints);
			cnc.constructHull(alpha);
			_hullPts = cnc._hullPts;
			_polygons = cnc._polygons;

			// Find perimeter

			// Find area
		}

	}
}

