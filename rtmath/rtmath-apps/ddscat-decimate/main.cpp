/* This program reads in a shape file and decimates / expands the grid by the 
* specified contraction / dilation factor. It also can write diel.tab-style 
* files for the different possible cases of filled / unfilled cells, using 
* a variety of techniques.
*/

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <iostream>
#include <sstream>
//#include <list>
#include <vector>
//#include <cstdlib>

#include <Ryan-Debug/debug.h>
#include <Ryan_Serialization/serialization.h>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

#include "../../rtmath/rtmath/denseMatrix.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/dielTabFile.h"
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/common_templates.h" // gives paramSet
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/error/error.h"

int main(int argc, char** argv)
{
	using namespace std;
	try {
		namespace po = boost::program_options;

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("input,i", po::value<string >(), 
			"Specify input shape file.")
			/*("inputddpar,p", po::value<string>(),
			"Specify input ddscat.par file")
			("inputdir,d", po::value<string>(),
			"Specify input directory")
			("base-dipole-spacing", po::value<double>(), 
			"Specify the base dipole spacing (microns)")*/
			("output,o", po::value<string>()->default_value("./decimated.shp"),
			"Specify output file")
			("twolevels", po::value<unsigned int>(), "Give output shape only two levels, "
			"with the resultant cells being filled or not using a threshold.")
			("factor,f", po::value<unsigned int>()->default_value(2),
			"scaling factor (in each direction)")
			;

		po::positional_options_description p;
		p.add("input",1);
		p.add("output",2);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);

		auto doHelp = [&desc](const std::string &message)
		{
			std::cerr << message << std::endl;
			std::cerr << desc << std::endl;
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");
		if (!vm.count("input")) doHelp("Need to specify input file\n");

		string input = vm["input"].as<string>();
		string output = vm["output"].as<string>();

		unsigned int scale = vm["factor"].as<unsigned int>();
		if (!scale) doHelp("factor must be an integer greater than zero");

		bool expand = (vm.count("expand")) ? true : false;

		bool hasThreshold = (vm.count("twolevels")) ? true : false;
		int threshold = 0;
		if (hasThreshold) threshold = vm["twolevels"].as<unsigned int>();

		// TODO: add in diel commands here

		namespace files = boost::filesystem;

		// Verify input file existence
		if (!files::exists(files::path(input))) doHelp("Input file does not exist");
		if (files::is_directory(files::path(input))) doHelp("Input file is a directory");
		//if (files::exists(files::path(output)))

		using namespace rtmath::ddscat;

		// Open the input file
		boost::shared_ptr<const shapefile> shp = boost::shared_ptr<const shapefile>(
			new rtmath::ddscat::shapefile(input));

		// Find min / max of points using shapestats
		//shapeFileStats::doQhull(false);
		//boost::shared_ptr<const shapeFileStats> sstats = boost::shared_ptr<const shapeFileStats>(
		//	new shapeFileStats(shp));

		// Use pcl to create a point cloud of the existing points
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloudDec (new pcl::PointCloud<pcl::PointXYZI>);

		// Insert points into the cloud
		cloud->points.resize(shp->numPoints);
		cloud->width = shp->numPoints;
		cloud->height = 1;

		cloudDec->points.reserve(shp->numPoints);
		for (size_t i=0; i< shp->numPoints; ++i)
		{
			cloud->points[i].x = shp->latticePts(i,0);
			cloud->points[i].y = shp->latticePts(i,1);
			cloud->points[i].z = shp->latticePts(i,2);
		}

		float resolution = (float) scale;
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

		octree.setInputCloud(cloud);
		octree.addPointsFromInputCloud();

		// Calculate decimation factor (max number of filled points within voxel)
//		unsigned int nMax = pow((unsigned int) scale,3);

		// The search point spans every voxel in the grid
		using namespace std;
		//cerr << shp->mins(0) << ", " << shp->maxs(0) << endl;
		//cerr << shp->mins(1) << ", " << shp->maxs(1) << endl;
		//cerr << shp->mins(2) << ", " << shp->maxs(2) << endl;

		for (float x=shp->mins(0); x <= shp->maxs(0); x += scale)
			for (float y=shp->mins(1); y <= shp->maxs(1); y += scale)
				for (float z=shp->mins(2); z <= shp->maxs(2); z+= scale)
				{
//					cerr << "X " << x << " Y " << y << " Z " << z << endl;
					pcl::PointXYZ searchPoint;
					searchPoint.x = x;
					searchPoint.y = y;
					searchPoint.z = z;

					// Aggregate points based on voxel. 
					std::vector<int> pointIdxVec;

					// Based on search point, find the total number of points in this voxel
					// Store information (and new dielectric index) into a new shapefile
					if (octree.voxelSearch (searchPoint, pointIdxVec))
//					octree.voxelSearch(searchPoint, pointIdxVec);
					{
						/*
						std::cerr << "Neighbors within voxel search at (" << searchPoint.x 
							<< " " << searchPoint.y 
							<< " " << searchPoint.z << ") " 
							<< pointIdxVec.size() << std::endl;
						
						for (size_t i = 0; i < pointIdxVec.size (); ++i)
							std::cout << "    " << cloud->points[pointIdxVec[i]].x 
							<< " " << cloud->points[pointIdxVec[i]].y 
							<< " " << cloud->points[pointIdxVec[i]].z << std::endl;
						*/

						// Scaling points to grid (starting grid at zero)
						float sx = (x - shp->mins(0)) / (float) scale;
						float sy = (y - shp->mins(1)) / (float) scale;
						float sz = (z - shp->mins(2)) / (float) scale;

						pcl::PointXYZI decPoint;
						decPoint.x = sx;
						decPoint.y = sy;
						decPoint.z = sz;
						decPoint.intensity = (hasThreshold) ? 
							((pointIdxVec.size() > (size_t) threshold) ? 1.0f : 0)
							: (float) pointIdxVec.size();

						if (decPoint.intensity)
							cloudDec->points.push_back(decPoint);
					}
/*
					int K = 2;

					std::vector<int> pointIdxNKNSearch;
					std::vector<float> pointNKNSquaredDistance;

					std::cerr << "K nearest neighbor search at (" << searchPoint.x 
					    << " " << searchPoint.y 
					    << " " << searchPoint.z
					    << ") with K=" << K << std::endl;

					if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
					{
						for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
							std::cerr << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
								<< " " << cloud->points[ pointIdxNKNSearch[i] ].y 
								<< " " << cloud->points[ pointIdxNKNSearch[i] ].z 
								<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
					}
*/
				}

		// Recalculate mean, and write the shapefile
		boost::shared_ptr<shapefile> shpout = boost::shared_ptr<shapefile>(
			new rtmath::ddscat::shapefile);
		//rtmath::ddscat::shapefile shpout = shp;
		shpout->desc = shp->desc;
		{ // Add descriptive naming
			std::ostringstream idname;
			idname << " with scaling factor " << scale;
			if (hasThreshold)
				idname << " and a filling threshold of " << threshold;
			std::string ids = idname.str();
			shpout->desc.append(ids);
		}
		shpout->a1 = shp->a1;
		shpout->a2 = shp->a2;
		shpout->a3 = shp->a3;
		shpout->d = shp->d;
		shpout->xd = shp->xd;
		shpout->Dielectrics = shp->Dielectrics;
		
		shpout->filename = output;
		shpout->numPoints = cloudDec->points.size();

		shpout->latticePts.resize(shpout->numPoints,3);
		shpout->latticePtsRi.resize(shpout->numPoints,3);

		using namespace boost::accumulators;
		// Finding the new mean, the easier way.
		accumulator_set<float, stats<tag::mean> > m_x, m_y, m_z;

		for (size_t i=0; i<shpout->numPoints; ++i)
		{
			shpout->latticePts(i,0) = cloudDec->points[i].x;
			shpout->latticePts(i,1) = cloudDec->points[i].y;
			shpout->latticePts(i,2) = cloudDec->points[i].z;

			shpout->latticePtsRi(i,0) = cloudDec->points[i].intensity;
			shpout->latticePtsRi(i,1) = cloudDec->points[i].intensity;
			shpout->latticePtsRi(i,2) = cloudDec->points[i].intensity;

			m_x(shpout->latticePts(i,0));
			m_y(shpout->latticePts(i,1));
			m_z(shpout->latticePts(i,2));
		}

		// Find min / max of points using shapestats
		//rtmath::ddscat::shapeFileStats sstatsout(shpout);
		//shpout->x0 = sstatsout.b_mean;
		//shpout->x0 = shp->x0;
		shpout->x0(0) = boost::accumulators::mean(m_x);
		shpout->x0(1) = boost::accumulators::mean(m_y);
		shpout->x0(2) = boost::accumulators::mean(m_z);

		shpout->write(output);

		// Calculate dielectric information using the 
		// selected dielectric pattern

		// Write the dielectric files

		// Write the new ddscat.par file (referencing new dielectrics)

		// Make a vtk file of the decimated shape

	} catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}
