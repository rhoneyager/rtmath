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

#include "../rtmath/error/error.h"
#include "../rtmath/Garrett/image.h"
#include "../rtmath/Garrett/mesh.h"
#include "../rtmath/Garrett/pclstuff.h"

namespace rtmath
{
	namespace Garrett
	{
		image::image()
		{
			_pc = boost::shared_ptr<pointContainer>
				(new pointContainer);
			_mesh = boost::shared_ptr<meshObj>
				(new meshObj);
		}

		image::~image()
		{
		}

		void image::read(const std::string &filename)
		{
			readPNG(filename);
		}

		void image::readPNG(const std::string &filename)
		{
			_pc->readPNG(filename);
		}

		void image::writePNG(const std::string &filename) const
		{
			_pc->writePNG(filename);
		}

		void image::writeROOTsurf(const std::string &filename) const
		{
			_pc->writeROOTsurf(filename);
		}

		void image::writeROOTzhist(const std::string &filename) const
		{
			_pc->writeROOTzhist(filename);
		}

		void image::readVTKpoints(const std::string &filename)
		{
			_pc->readVTKpoints(filename);
		}

		void image::writeVTKpoints(const std::string &filename) const
		{
			_pc->writeVTKpoints(filename);
		}

		void image::readMesh(const std::string &filename)
		{
		}

		void image::writeMesh(const std::string &filename) const
		{
		}

		void image::readPCD(const std::string &filename)
		{
			_pc->readPCD(filename);
		}

		void image::writePCD(const std::string &filename) const
		{
			_pc->writePCD(filename);
		}

		void image::_meshPCL() 
		{
		}


	}
}

