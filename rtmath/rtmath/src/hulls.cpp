#include "../rtmath/Stdafx.h"
#pragma warning( disable : 4996 )
#pragma warning( disable : 4305 )
#pragma warning( disable : 4244 )

#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>

// Need to disable PCL warnings that are extraneous
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#pragma warning(push)
#pragma warning( disable : 4521 )
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/Vertices.h>
//#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
//#include <pcl/kdtree/kdtree.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/gp3.h>
//#include <pcl/surface/grid_projection.h>
//#include <pcl/surface/marching_cubes.h>
//#include <pcl/surface/mls.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/project_inliers.h>
//#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/PolygonMesh.h>
#pragma warning(pop)

#include "../rtmath/ddscat/hulls.h"
#include "../rtmath/error/error.h"

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

namespace rtmath
{
	namespace ddscat
	{
		class hullData
		{
		public:
			hullData()
				: volume(0), surfarea(0), hull_enabled(true)
			{
				points = pcl::PointCloud<pcl::PointXYZ>::Ptr(
					new pcl::PointCloud<pcl::PointXYZ>);
				hullPts = pcl::PointCloud<pcl::PointXYZ>::Ptr(
					new pcl::PointCloud<pcl::PointXYZ>);
			}
			virtual ~hullData()
			{
				//points = nullptr;
				//polygons.clear();
				//hullPts = nullptr;
			}
			pcl::PointCloud<pcl::PointXYZ>::Ptr points;
			std::vector< pcl::Vertices > polygons;
			mutable pcl::PointCloud<pcl::PointXYZ>::Ptr hullPts;
			double volume, surfarea;
			bool hull_enabled;
		};

		void writeVTKpoints(const std::string &filename, const pcl::PointCloud<pcl::PointXYZ> &src)
		{
			sensor_msgs::PointCloud2 pbc;
			pcl::toROSMsg(src, pbc);
			pcl::io::saveVTKFile(filename.c_str(),pbc);
		}
		*/
		hull::hull()
		{
			_p = boost::shared_ptr<hullData>(new hullData);
			_points = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
			_hullPts = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
		}

		hull::~hull() { }

		/*
		hull::hull(const pcl::PointCloud<pcl::PointXYZ>::Ptr &backend)
		{
			_p = boost::shared_ptr<hullData>(new hullData);
			_p->points = backend;
			_p->hullPts = backend;
		}
		*/

		hull::hull(const Eigen::Matrix<float, Eigen::Dynamic, 3> &backend)
		{
			_p = boost::shared_ptr<hullData>(new hullData);
			_p->points->resize(backend.rows());
			_p->hullPts->resize(backend.rows());
			auto map = _p->points->getMatrixXfMap(3,4,0);
			map = backend.transpose();
			auto hullmap = _p->hullPts->getMatrixXfMap(3,4,0);
			hullmap = backend.transpose();
		}

		void hull::writeVTKhull(const std::string &filename) const
		{
			pcl::PolygonMesh triangles;
			triangles.polygons = _p->polygons;
			sensor_msgs::PointCloud2 pbc;
			pcl::toROSMsg( *(_p->hullPts.get()), pbc);
			triangles.cloud = pbc;
			
			//generateMeshGreedy(*this,cloud,triangles);
			pcl::io::saveVTKFile (filename.c_str(), triangles);
		}

		void hull::hullEnabled(bool val) { _p->hull_enabled = val; }

		bool hull::hullEnabled() const { return _p->hull_enabled; }

		//convexHull::convexHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src)
		//	: hull(src) {}

		convexHull::convexHull(const Eigen::Matrix<float, Eigen::Dynamic, 3>& src) : hull(src) {}

		convexHull::~convexHull() { }

		void convexHull::constructHull()
		{
			_p->hullPts->clear();

			pcl::ConvexHull<pcl::PointXYZ> chull;
			// //Dimension is not set because it is also used for 2d shapes
			chull.setDimension(3); // Dimension 3 fails for dendrites
			chull.setComputeAreaVolume(true);
			chull.setInputCloud (_p->points);
			chull.reconstruct (*(_p->hullPts.get()), _p->polygons);
			_p->volume = chull.getTotalVolume();
			_p->surfarea = chull.getTotalArea();
		}

		double convexHull::maxDiameter() const
		{
			Eigen::Matrix<float, 2, 3> range;
			return maxDiameter(range);
		}

		double convexHull::maxDiameter(Eigen::Matrix<float, 2, 3> &range) const
		{
			pcl::PointXYZ min, max;

			auto fMaxDiameter = [&](pcl::PointXYZ &min, pcl::PointXYZ &max) -> double
			{
				double maxD = 0;
				pcl::PointCloud<pcl::PointXYZ>::Ptr base;
				if (hullEnabled()) base = _p->hullPts;
				else base = _p->points;

				// This function is too slow
				//maxD = pcl::getMaxSegment(*base, min, max);

				for (auto it = base->begin(); it != base->end(); ++it)
				{
					for (auto ot = base->begin(); ot != base->end(); ++ot)
					{
						double d = (it->x - ot->x) * (it->x - ot->x);
						d += (it->y - ot->y,2.0f) * (it->y - ot->y,2.0f);
						d += (it->z - ot->z,2.0f) * (it->z - ot->z,2.0f);
						if (d > maxD)
						{
							maxD = d;
							max = *it;
							min = *ot;
						}
					}
				}
				return sqrt(maxD);
			};

			double md = fMaxDiameter(min,max);
			range.block<1,3>(0,0) = max.getVector3fMap().transpose();
			range.block<1,3>(1,0) = min.getVector3fMap().transpose();
			return md;
		}

		/*
		double convexHull::maxDiameter(pcl::PointXYZ& min, pcl::PointXYZ& max) const
		{
			double maxD = 0;
			pcl::PointCloud<pcl::PointXYZ>::Ptr base;
			if (hullEnabled()) base = _p->hullPts;
			else base = _p->points;

			// This function is too slow
			//maxD = pcl::getMaxSegment(*base, min, max);

			for (auto it = base->begin(); it != base->end(); ++it)
			{
				for (auto ot = base->begin(); ot != base->end(); ++ot)
				{
					double d = (it->x - ot->x) * (it->x - ot->x);
					d += (it->y - ot->y,2.0f) * (it->y - ot->y,2.0f);
					d += (it->z - ot->z,2.0f) * (it->z - ot->z,2.0f);
					if (d > maxD)
					{
						maxD = d;
						max = *it;
						min = *ot;
					}
				}
			}
			return sqrt(maxD);
		}
		*/

		double hull::volume() const { return _p->volume; }

		double hull::surfaceArea() const { return _p->surfarea; }

		//concaveHull::concaveHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src)
		//	: hull(src) { }

		concaveHull::concaveHull(const Eigen::Matrix<float, Eigen::Dynamic, 3>& src) : hull(src) {}

		concaveHull::~concaveHull() { }
		
		void concaveHull::constructHull(double alpha)
		{
			_p->hullPts->clear();

			pcl::ConcaveHull<pcl::PointXYZ> chull;
			chull.setDimension(3);
			chull.setInputCloud (_p->points);
			chull.setAlpha(alpha);
			chull.reconstruct(*(_p->hullPts.get()), _p->polygons);

			//_findVS();
		}


	}
}

