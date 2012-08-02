#include "../rtmath/Stdafx.h"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <set>
#include <map>

#include <Magick++.h>

#include <boost/filesystem.hpp>

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

#include "../rtmath/ddscat/hulls.h"

#pragma GCC diagnostic pop


#include "../rtmath/Garrett/image.h"
#include "../rtmath/Garrett/mesh.h"

namespace rtmath
{
	namespace Garrett
	{
		image::image()
		{
		}

		image::~image()
		{
		}

		image::image(const std::string &filename)
		{
		}

		void image::readPNG(const std::string &filename)
		{
		}

		void image::writePNG(const std::string &filename) const
		{
		}

		void image::readPoints(const std::string &filename)
		{
		}

		void image::writePoints(const std::string &filename) const
		{
		}

		void image::readMesh(const std::string &filename)
		{
		}

		void image::writeMesh(const std::string &filename) const
		{
		}

		void image::_meshPCL() 
		{
		}


	}
}

