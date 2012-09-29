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
#include "../rtmath/ddscat/hulls.h"
#include "../rtmath/error/error.h"
#include "../rtmath/PCLlink.h"

#pragma GCC diagnostic pop

namespace
{
	/*
	void generateMeshMarching(const rtmath::ddscat::hull &h, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PolygonMesh &triangles)
	{
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
		mls.setComputeNormals (true);
		mls.setInputCloud (cloud);
		mls.setPolynomialFit (true);
		mls.setSearchMethod (tree);
		mls.setSearchRadius (1.0);
		mls.process (*cloud_with_normals);
		//
		tree->setInputCloud (cloud);


		pcl::PolygonMesh mesh;

		// normals should not contain the point normals + surface curvatures

		
		// Create search tree*
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
		tree2->setInputCloud (cloud_with_normals);

		// Initialize objects
		pcl::MarchingCubes<pcl::PointNormal> mc;
		//pcl::PolygonMesh triangles;

		// Set parameters
		mc.setIsoLevel(0.5);
		mc.setGridResolution(50, 50, 50);
		mc.setSearchMethod(tree2);
		mc.setInputCloud(cloud_with_normals);

		mc.reconstruct (triangles);

	}
	void generateMeshGridProjection(const rtmath::ddscat::hull &h, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PolygonMesh &triangles)
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

		// normals should not contain the point normals + surface curvatures

		// Concatenate the XYZ and normal fields*
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
		// cloud_with_normals = cloud + normals

		// Create search tree*
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
		tree2->setInputCloud (cloud_with_normals);

		// Initialize objects
		pcl::GridProjection<pcl::PointNormal> gbpolygon;
		//pcl::PolygonMesh triangles;

		// Set parameters
		gbpolygon.setResolution(0.005);
		gbpolygon.setPaddingSize(1);
		//gbpolygon.setNearestNeighborNum(h.maxNearestNeighbors);
		gbpolygon.setMaxBinarySearchLevel(10);

		// Get result
		gbpolygon.setInputCloud(cloud_with_normals);
		gbpolygon.setSearchMethod(tree2);
		gbpolygon.reconstruct(triangles);
	}

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

		// normals should not contain the point normals + surface curvatures

		// Concatenate the XYZ and normal fields*
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
		// cloud_with_normals = cloud + normals

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
*/
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
			_volume = 0;
			_surfarea = 0;
			hull_enabled = true;
		}

		hull::~hull()
		{
		}

		hull::hull(const std::vector<matrixop> &backend)
		{
			_points = backend;
			_hullPts = backend;
			_volume = 0;
			_surfarea = 0;
		}

		void hull::writeVTKhull(const std::string &filename) const
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
			const size_t N = _hullPts.size();
			if (!N) throw rtmath::debug::xBadInput("VTK hull write needs populated hull points.");
			cloud->reserve(N);
			
			for (auto it = _hullPts.begin(); it != _hullPts.end(); it++)
			{
				const double x = it->get(2,0,0);
				const double y = it->get(2,0,1);
				const double z = it->get(2,0,2);
				cloud->push_back(pcl::PointXYZ(x,y,z));
			}



			pcl::PolygonMesh triangles;
			triangles.polygons = _polygons;
			sensor_msgs::PointCloud2 pbc;
			pcl::toROSMsg(*cloud, pbc);
			triangles.cloud = pbc;
			
			//generateMeshGreedy(*this,cloud,triangles);
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
//			chull.setDimension(3); // Deimension 3 fails for dendrites
			chull.setComputeAreaVolume(true);
			chull.setInputCloud (cloud);
			chull.reconstruct (*cloud_hull, _polygons);
			_volume = chull.getTotalVolume();
			_surfarea = chull.getTotalArea();

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
			const std::vector<matrixop>* base = nullptr;
			if (hull_enabled) base = &_hullPts;
			else base = &_points;
			for (auto it = base->begin(); it != base->end(); it++)
			{
				const double ix = it->get(2,0,0);
				const double iy = it->get(2,0,1);
				const double iz = it->get(2,0,2);
				for (auto ot = it + 1; ot != base->end(); ot++)
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

		double hull::volume() const
		{
			return _volume;
		}

		double hull::surface_area() const
		{
			return _surfarea;
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
			chull.reconstruct (*cloud_hull, _polygons);

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

			_findVS();
		}

		void concaveHull::_findVS()
		{
			throw rtmath::debug::xUnimplementedFunction();
		}

	}
}

