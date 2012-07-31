#include "../rtmath/Stdafx.h"
#pragma warning( disable : 4996 )
#pragma warning( disable : 4305 )
#pragma warning( disable : 4244 )

#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>

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

		//* normals should not contain the point normals + surface curvatures

		
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
	*/
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

		//* normals should not contain the point normals + surface curvatures

		// Concatenate the XYZ and normal fields*
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
		//* cloud_with_normals = cloud + normals

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
			searchRadius = 1.5;
			Mu = 1;
			minAngle = 0;
			maxAngle = 3.14159/2; // TODO: replace with boost def
			maxSurfAngle = 3.14159/2;
			maxNearestNeighbors = 200;
			normalConsistency = true;
			_volume = 0;
			_surfarea = 0;
		}

		hull::~hull()
		{
		}

		hull::hull(const std::vector<matrixop> &backend)
		{
			hull();
			searchRadius = 1.5;
			Mu = 1;
			minAngle = 0;
			maxAngle = 3.14159/2; // TODO: replace with boost def
			maxSurfAngle = 3.14159/2;
			maxNearestNeighbors = 200;
			normalConsistency = true;
			_points = backend;
			_hullPts = backend;
			_volume = 0;
			_surfarea = 0;
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
			chull.setDimension(3);
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
			// Using _hullPts and _polygons, determine surface area (easy),
			// and volume (hard)
			
			_volume = 0;
			_surfarea = 0;
			// First, need to find bounds on iteration and matrix sizes
			using namespace boost::accumulators;
			accumulator_set<size_t, stats<tag::min, tag::max> > bnd_x, bnd_y, bnd_z;
			for (auto it = _hullPts.begin(); it != _hullPts.end(); it++)
			{
				double x = it->get(2,0,0);
				double y = it->get(2,1,0);
				double z = it->get(2,2,0);

				bnd_x((size_t) x);
				bnd_y((size_t) y);
				bnd_z((size_t) z);
			}
			const size_t y_range = boost::accumulators::max(bnd_y) - boost::accumulators::min(bnd_y);
			const size_t z_range = boost::accumulators::max(bnd_z) - boost::accumulators::min(bnd_z);
			const size_t x_min = boost::accumulators::min(bnd_x);
			const size_t x_range = boost::accumulators::max(bnd_x) - x_min;
			// Need the center points for concave volume determination
			const double x_c = x_min + 0.5 * x_range;
			const double y_c = boost::accumulators::min(bnd_y) + 0.5 * y_range;
			const double z_c = boost::accumulators::min(bnd_z) + 0.5 * z_range;

			// Iterate over polygons for both surface area and volume elements
/*			pcl::PolygonMesh triangles;
			triangles.polygons = _polygons;
			sensor_msgs::PointCloud2 pbc;
			pcl::toROSMsg(*cloud, pbc);
			triangles.cloud = pbc;
			
*/

			for (auto it = _polygons.begin(); it != _polygons.end(); it++)
			{
				// m contains the det matrix for volume computation
				double m[4][4];
				for (size_t i = 0; i < (*it).vertices.size(); i++)
				{
					size_t v = (*it).vertices[i]; // v will be the index
					// _hullPts[v] has the coords
					m[i][0] = _hullPts[v].get(2,0,0);
					m[i][1] = _hullPts[v].get(2,1,0);
					m[i][2] = _hullPts[v].get(2,2,0);
					m[i][3] = 1.0;
				}
				m[3][0] = x_c;
				m[3][1] = y_c;
				m[3][2] = z_c;
				m[3][3] = 1.0;

				// This matrix makes volume and surface area determination easy!

				matrixop vd(2,4,4);
				vd.fromDoubleArray(&m[0][0]);
				_volume += vd.det() / 6.0;

				double ss = (m[0][1]*m[1][2]) - (m[0][2]*m[1][1]);
				ss -= (m[0][0]*m[1][2]) - (m[0][2]*m[1][0]);
				ss += (m[0][0]*m[1][1]) - (m[0][1]*m[1][0]);
				_surfarea += abs(ss);

				// Sign is also critical, as the vertex segments overlap 
				// in x. Conveniently, the polygons may never overlap in 3d space.
			}
		}

	}
}

