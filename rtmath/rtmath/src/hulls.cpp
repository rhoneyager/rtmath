#include "../rtmath/Stdafx.h"
#pragma warning( disable : 4996 )
#pragma warning( disable : 4305 )
#pragma warning( disable : 4244 )

#include <algorithm>
#include <boost/shared_ptr.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/PolygonMesh.h>

//#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/ddscat/hulls.h"

namespace
{
	void generateMeshGreedy(const rtmath::ddscat::hull &h, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PolygonMesh &triangles)
	{
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		//
		tree->setInputCloud (cloud);
		n.setInputCloud (cloud);
		n.setSearchMethod (tree);
		n.setKSearch (20);
		n.compute (*normals);

		//* normals should not contain the point normals + surface curvatures

		// Concatenate the XYZ and normal fields*
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
		//* cloud_with_normals = cloud + normals

		// Create search tree*
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
		tree2->setInputCloud (cloud_with_normals);

		// Initialize objects
		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
		//pcl::PolygonMesh triangles;

		// Set the maximum distance between connected points (maximum edge length)
		gp3.setSearchRadius (h.searchRadius);

		// Set typical values for the parameters
		gp3.setMu (h.Mu);
		gp3.setMaximumNearestNeighbors (h.maxNearestNeighbors);
		//gp3.setMaximumSurfaceAngle(M_PI/2); // 45 degrees
		gp3.setMaximumSurfaceAngle(h.maxSurfAngle);
		gp3.setMinimumAngle(h.minAngle);
		gp3.setMaximumAngle(h.maxAngle);
  
		gp3.setNormalConsistency(h.normalConsistency);

		// Get result
		gp3.setInputCloud (cloud_with_normals);
		gp3.setSearchMethod (tree2);
		gp3.reconstruct (triangles);

		// Additional vertex information
		std::vector<int> parts = gp3.getPartIDs();
		std::vector<int> states = gp3.getPointStates();

	}
}

namespace rtmath
{
	namespace ddscat
	{
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

		hull::hull()
		{
			searchRadius = 5.0;
			Mu = 2.2;
			minAngle = 0;
			maxAngle = 3.14159/2; // TODO: replace with boost def
			maxSurfAngle = 3.14159/2;
			maxNearestNeighbors = 200;
			normalConsistency = false;
		}

		hull::~hull()
		{
		}

		hull::hull(const std::vector<matrixop> &backend)
		{
			hull();
			searchRadius = 5.0;
			Mu = 2.2;
			minAngle = 0;
			maxAngle = 3.14159/2; // TODO: replace with boost def
			maxSurfAngle = 3.14159/2;
			maxNearestNeighbors = 200;
			normalConsistency = false;
			_points = backend;
			_hullPts = backend;
		}

		void hull::writeVTKhull(const std::string &filename) const
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
			const size_t N = _hullPts.size();
			cloud->reserve(N);
			
			for (auto it = _hullPts.begin(); it != _hullPts.end(); it++)
			{
				const double x = it->get(2,0,0);
				const double y = it->get(2,0,1);
				const double z = it->get(2,0,2);
				cloud->push_back(pcl::PointXYZ(x,y,z));
			}

			pcl::PolygonMesh triangles;
			generateMeshGreedy(*this,cloud,triangles);
			pcl::io::saveVTKFile (filename.c_str(), triangles);
		}

		convexHull::convexHull(const std::vector<matrixop> &src)
			: hull(src)
		{
		}

		convexHull::~convexHull()
		{
		}

		void convexHull::constructHull()
		{
			_hullPts.clear();

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

			const size_t N = _points.size();
			cloud->reserve(N);

			// Cannot use std::copy to populate the cloud, as type conversion needs to occur
			
			for (auto it = _points.begin(); it != _points.end(); it++)
			{
				const double x = it->get(2,0,0);
				const double y = it->get(2,0,1);
				const double z = it->get(2,0,2);
				cloud->push_back(pcl::PointXYZ(x,y,z));
			}

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::ConvexHull<pcl::PointXYZ> chull;
			chull.setDimension(3);
			chull.setInputCloud (cloud);
			chull.reconstruct (*cloud_hull);

			// Write to _hullPts
			_hullPts.reserve(cloud_hull->size());
			for (auto it = cloud_hull->begin(); it != cloud_hull->end(); it++)
			{
				matrixop p(2,1,3);
				p.set(it->x,2,0,0);
				p.set(it->y,2,0,1);
				p.set(it->z,2,0,2);

				_hullPts.push_back(std::move(p));
			}
		}

		double convexHull::maxDiameter() const
		{
			double maxD = 0;
			for (auto it = _hullPts.begin(); it != _hullPts.end(); it++)
			{
				const double ix = it->get(2,0,0);
				const double iy = it->get(2,0,1);
				const double iz = it->get(2,0,2);
				for (auto ot = it + 1; ot != _hullPts.end(); ot++)
				{
					const double ox = ot->get(2,0,0);
					const double oy = ot->get(2,0,1);
					const double oz = ot->get(2,0,2);

					double d = sqrt(pow(ix-ox,2.0)+pow(iy-oy,2.0)+pow(iz-oz,2.0));
					if (d > maxD) maxD = d;
				}
			}

			return maxD;
		}

		concaveHull::concaveHull(const std::vector<matrixop> &src)
			: hull(src)
		{
		}

		concaveHull::~concaveHull()
		{
		}

		void concaveHull::constructHull(double alpha)
		{
			_hullPts.clear();

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

			const size_t N = _points.size();
			cloud->reserve(N);

			// Cannot use std::copy to populate the cloud, as type conversion needs to occur
			
			for (auto it = _points.begin(); it != _points.end(); it++)
			{
				const double x = it->get(2,0,0);
				const double y = it->get(2,0,1);
				const double z = it->get(2,0,2);
				cloud->push_back(pcl::PointXYZ(x,y,z));
			}

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::ConcaveHull<pcl::PointXYZ> chull;
			chull.setDimension(3);
			chull.setInputCloud (cloud);
			chull.setAlpha(alpha);
			chull.reconstruct (*cloud_hull);

			// Write to _hullPts
			_hullPts.reserve(cloud_hull->size());
			for (auto it = cloud_hull->begin(); it != cloud_hull->end(); it++)
			{
				matrixop p(2,1,3);
				p.set(it->x,2,0,0);
				p.set(it->y,2,0,1);
				p.set(it->z,2,0,2);

				_hullPts.push_back(std::move(p));
			}
		}

	}
}

