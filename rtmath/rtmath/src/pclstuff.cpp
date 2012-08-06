#include "../rtmath/Stdafx.h"

#include "../rtmath/ROOTlink.h"
#include "../rtmath/MagickLINK.h"

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
		pointContainer::pointContainer()
			: cloud(new pcl::PointCloud<pcl::PointXYZ>)
		{
		}

		pointContainer::~pointContainer()
		{
		}

		void pointContainer::readPCD(const std::string &filename)
		{
			if (pcl::io::loadPCDFile<pcl::PointXYZ>
				(filename.c_str(), *cloud) == -1)
			{
				throw rtmath::debug::xUnknownFileFormat(filename.c_str());
			}
		}

		void pointContainer::writePCD(const std::string &filename) const
		{
			pcl::io::savePCDFileASCII(filename.c_str(), *cloud);
		}

		void pointContainer::readPNG(const std::string &filename)
		{
			using namespace Magick;
			using namespace std;
			cloud->points.clear();

			// First, use ImageMagick to read the PNG file
			Image img;
			img.read(filename.c_str());
			// Define the view area 
			int start_x = 0, start_y = 0;
			int size_x = img.columns(), size_y = img.rows();

			cout << "Image is " << size_x << " x " << size_y << endl;

			cloud->points.reserve((size_t) (size_x*size_y) );
			//pixels = pcache.get(start_x, start_y, size_x, size_y);
			// pixels now contains the pixels accessable by pointer
			// pixels can be rescaled in brightness values, or 
			// can have the dimensions transformed in 
			// the point cloud array. For now, though, just 
			// translate directly into the PCL format.
			for (size_t row=0;row< (size_t) size_x;row++)
			{
				for (size_t col=0;col< (size_t) size_y;col++)
				{
					// Cast upwards to grayscale and 
					// extract shade
					ColorGray c = img.pixelColor(row,col);
					// zval ranges from 0 to 1.0
					double zval = c.shade();
					if (zval < 0.0005) continue;

					// Set point in array
					pcl::PointXYZ point;
					point.x = (double) row;
					point.y = (double) (size_y - col);
					point.z = zval * 100;

					cloud->points.push_back(point);
				}
			}
			cout << "Processed " << cloud->points.size() << " points" << endl;
		}

		void pointContainer::writePNG(const std::string &filename) const
		{
		}

		void pointContainer::readVTKpoints(const std::string &filename)
		{
		}

		void pointContainer::writeVTKpoints(const std::string &filename) const
		{
			sensor_msgs::PointCloud2 pbc;
			pcl::toROSMsg(*cloud, pbc);
			pcl::io::saveVTKFile(filename.c_str(),pbc);
		}

		void pointContainer::writeROOTzhist(const std::string &filename) const
		{
			using namespace std;
			const size_t nP = cloud->points.size();
			boost::shared_ptr<TCanvas> tc(new TCanvas("c","Surface", 0, 0, 700, 600));
			boost::shared_ptr<TH1D> tp(new TH1D());
			//tp->Set(nP);
			tp->SetBins(50,0,80);
			//tp->SetBins(410,0,410,640,0,640);

			size_t n=0;
			for (auto it = cloud->points.begin(); it != cloud->points.end(); it++, n++)
			{
				//tp->Fill(it->x,it->y);
				tp->Fill(it->z);
			}
			gStyle->SetPalette(1);
			tp->Draw();
			tp->SetTitle(string("Z brightness histogram for ").append(filename).c_str());
			tp->GetXaxis()->SetTitle("Brightness");
			tp->GetXaxis()->CenterTitle();
			tp->GetYaxis()->SetTitle("Frequency");
			tp->GetYaxis()->CenterTitle();

			tc->SaveAs(filename.c_str());
		}


		void pointContainer::writeROOTsurf(const std::string &filename) const
		{
			using namespace std;
			// cloud->points
			const size_t nP = cloud->points.size();
			boost::shared_ptr<TCanvas> tc(new TCanvas("c","Surface", 0, 0, 700, 600));
			boost::shared_ptr<TH2D> tp(new TH2D());
			//tp->Set(nP);
			tp->SetBins(410,0,410,640,0,640);

			size_t n=0;
			for (auto it = cloud->points.begin(); it != cloud->points.end(); it++, n++)
			{
				//tp->Fill(it->x,it->y);
				tp->Fill(it->x,it->y,it->z);
			}
			gStyle->SetPalette(1);
			tp->Draw("CONTZ");
			tp->SetTitle(string("Surface ").append(filename).c_str());
			tp->GetXaxis()->SetTitle("x");
			tp->GetXaxis()->CenterTitle();
			tp->GetYaxis()->SetTitle("y");
			tp->GetYaxis()->CenterTitle();
			tp->GetZaxis()->SetTitle("z");
			tp->GetZaxis()->CenterTitle();

			tc->SaveAs(filename.c_str());
		}

	}
}


