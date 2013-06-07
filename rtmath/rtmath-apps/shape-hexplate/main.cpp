/* This program generates hexagonal plates with proper aspect ratios.
 */

#pragma warning( push )
#pragma warning( disable : 4996 ) // Dumb boost uuid warning
#pragma warning( disable : 4800 ) // forcing non-bool type to true or false
#include <cmath>
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <set>

#pragma warning( disable : 4521 ) // Deprecated declaration
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>

//#include "../../rtmath/rtmath/VTKlink.h"

#include <boost/program_options.hpp>
#include <boost/shared_array.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>

#include "../../rtmath/rtmath/denseMatrix.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/error/debug.h"

void rangeSectorSnowflake(const float rhw[3], const float c[3], 
	float mins[3], float maxs[3])
{
	using namespace std;
	// This can be narrowed, but it's irrelevent for now
	float rMax = max (max( rhw[0]+1, rhw[1]+1), rhw[2]+1);
	for (size_t i=0; i<3; i++)
	{
		mins[i] = c[i] - rMax;
		maxs[i] = c[i] + rMax;
	}
}

void fillSectorSnowflake(rtmath::denseMatrix &dm, const float rhw[3], const float c[3], 
	const float n[3], const std::set<float> &angles, bool sign)
{
	using namespace std;
	using namespace rtmath;
	// Draw half-ellipses specified by the inputs to make sector snowflake shapes
	// For speed, use the PCL. Create a uniform grid, then filter based on constraints.
	// Constraints include the angle (plane normal) and the bounding sphere.

	// Build the base arm shape
	float mins[3], maxs[3];
	rangeSectorSnowflake(rhw, c, mins, maxs);
	float sizeX = maxs[0] - mins[0] + 1.0f;
	float sizeY = maxs[1] - mins[1] + 1.0f;
	float sizeZ = maxs[2] - mins[2] + 1.0f;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cBase(new pcl::PointCloud<pcl::PointXYZ>);
	cBase->reserve( (size_t) (sizeX*sizeY*sizeZ));

	//rtmath::denseMatrix mask((size_t) sizeX,(size_t) sizeY,(size_t) sizeZ);
	for (float x = mins[0]; x <=maxs[0]; x++)
	{
		for (float y = mins[1]; y <=maxs[1]; y++)
		{
			for (float z = mins[2]; z <=maxs[2]; z++)
			{
				if (0.25 >= pow((x-c[0])/rhw[0],2.0f) + pow((y-c[1])/rhw[1],2.0f) + pow((z-c[2])/rhw[2],2.0f))
				{
					if (x-c[0] >= 0)
					{
						cBase->push_back(pcl::PointXYZ(x-c[0],y-c[1],z-c[2]));
						if (x-mins[0] > 1200 || y-mins[1] > 1200 || z-mins[2] > 1200)
						{
							cerr << "Error\n";
						}
						//dm.set(x-mins[0],y-mins[1],z-mins[2],true);
					}
				}
			}
		}
	}
	
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
	tree->setInputCloud (cBase);

	float nx = n[0] * boost::math::constants::pi<float>() / 180.0f;
	float ny = n[1] * boost::math::constants::pi<float>() / 180.0f;
	float nz = n[2] * boost::math::constants::pi<float>() / 180.0f;
	// Taking the base arm and duplicating it through rotations. Note: we cannot just copy from the base to 
	// the destination since we get rounding errors. A better method is to assign a dense matrix to the original 
	// copy and go through the rotations, comparing to the copy.

	// Now, take the base arm and duplicate it into the appropriate locations in dm
	for (auto it = angles.begin(); it != angles.end(); ++it)
	{
		// rhw are the semimajor axes lengths
		// c is the center
		// n is the collection of rotations relative to the +x direction
		// each angle is the angle beta for the shape rotation
		float na = *it * boost::math::constants::pi<float>() / 180.0f;

		// Construct corresponding rotation and inverse rotation matrices
		Eigen::Matrix3f Ref, Rinv; // Ref should be YXZ?
		Ref = Eigen::AngleAxisf(nx, Eigen::Vector3f::UnitZ())
			* Eigen::AngleAxisf(ny, Eigen::Vector3f::UnitX())
			* Eigen::AngleAxisf(nz+na, Eigen::Vector3f::UnitY());

		Rinv = Ref.inverse();
		cout << Rinv;
		cout << endl;



		// Test each possible point and run it through the inverse rotation matrix. If it is within a  
		// small distance (one dipole spacing) of an original point, mark the location as filled.
		for (float x = mins[0]; x <=maxs[0]; x++)
		{
			for (float y = mins[1]; y <=maxs[1]; y++)
			{
				for (float z = mins[2]; z <=maxs[2]; z++)
				{
					float xc = x-c[0], yc = y-c[1], zc = z - c[2];
					auto res = Rinv * Eigen::Vector3f(xc,yc,zc);
					float xp = res(0,0), yp = res(1,0), zp = res(2,0);
					pcl::PointXYZ testpoint(xp,yp,zp);
					vector<int> k_indices;
					vector<float> d_sq;

//#pragma warning( push )
//#pragma warning( disable : 4244 ) // Irritating PCL double / float stuff caused by this function
					tree->radiusSearch(testpoint,1.0, k_indices, d_sq);
//#pragma warning( pop )

					if (d_sq.size())
					{
						dm.set((size_t) (x-mins[0]+c[0]),(size_t) (y-mins[1]+c[1]),(size_t) (z-mins[2]+c[2]),sign);
					}

				}
			}
		}


	}
	
}

struct fillSet
{
	float rhw[3], c[3], n[3];
	std::set<float> angles;
	bool sign;
};

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-shape-hexplate\n\n";
		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("fill", -1);

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("aeff,a", po::value<double>(), "Effective radius (um)")
			("diameter", po::value<double>(), "Diameter (um)")
			("thickness", po::value<double>(), "Thickness (um)")
			("dipole-spacing,d", po::value<double>(), "Base dipole spacing along horizontal plane (um)")
			("aspect-ratio", po::value<double>(), "Force a given aspect ratio (width / depth). "
			 "Otherwise, use the default aspect ratio formula.")
			("scale-aspect-ratio", po::value<double>()->default_value(1.0), 
			 "Scale the default aspect ratio formula by the given factor.")
			("output,o", po::value<string>(), "This is the output filename. If unspecified, write to stdout.")
			("title", po::value<string>()->default_value("shape-hexplate generated shape"), "The description enclosed in the shape file")
			("a1", po::value<string>()->default_value("1,0,0"), "The a1 vector in csv form")
			("a2", po::value<string>()->default_value("0,1,0"), "The a2 vector in csv form")
//			("d", po::value<string>()->default_value("1,1,1"), "Dipole scaling factor")
			;

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);    

		if (vm.count("help") || argc == 1) {
			cerr << desc << "\n";
			return 1;
		}

		ostream *pout = nullptr;
		ofstream ofile;
		if (vm.count("output"))
		{
			ofile.open(vm["output"].as<string>().c_str());
			pout = &ofile;
		} else {
			pout = &cout;
		}
		ostream &out = *pout;

		double fixedAR = 0;
		if (vm.count("aspect-ratio")) fixedAR = vm["aspect-ratio"].as<double>();
		double scaleAR = vm["scale-aspect-ratio"].as<double>();

		bool calcAeff = false, calcDiam = false, calcThickness = false;
		double aeff = 0;
		double diameter = 0;
		double thickness = 0;
		
		if (vm.count("aeff")) aeff = vm["aeff"].as<double>();
		else calcAeff = true;

		if (vm.count("diameter")) diameter = vm["diameter"].as<double>();
		else calcDiam = true;

		if (vm.count("thickness")) thickness = vm["thickness"].as<double>();
		else calcThickness = true;

		double dSpacing = vm["dipole-spacing"].as<double>();

		// Check for parameter (value) clashes.
		
		// Fill in any missing values.

		// Do three passes. First pass allows for dimensioning and memory allocation.
		// Second pass fills in the shape into memory. Dense memory layout is selected, since we are dealing with rectangles.
		// Third pass writes the dipoles to disk
		using namespace boost::accumulators;
		accumulator_set<float, stats<tag::min, tag::max> > sx, sy, sz;
		accumulator_set<float, stats<tag::sum> > svolmax;
		cerr << "Pass 1" << endl;
		for (auto it = vfills.begin(); it != vfills.end(); ++it)
		{
			float imins[3], imaxs[3];
			rangeSectorSnowflake(it->rhw,it->c,imins,imaxs);

			sx(imins[0]);
			sx(imaxs[0]);
			sy(imins[1]);
			sy(imaxs[1]);
			sz(imins[2]);
			sz(imaxs[2]);
			svolmax( abs( (imaxs[0]-imins[0]+1) * (imaxs[1]-imins[1]+1) * (imaxs[2]-imins[2]+1) ) );
		}

		// If an input file is specified, load it here and determine its statistics for the min / max dimensioning.
		rtmath::ddscat::shapefile inshp;
		if (vm.count("input"))
		{
			inshp.read(vm["input"].as<string>());
			rtmath::ddscat::shapeFileStats::doQhull(false);
			rtmath::ddscat::shapeFileStats inStats(inshp);
			sx(inStats.b_min(0));
			sx(inStats.b_max(0));
			sy(inStats.b_min(1));
			sy(inStats.b_max(1));
			sz(inStats.b_min(2));
			sz(inStats.b_max(2));
		}

		// svolmax now has the max amount of memory required for processing.
		size_t sizeX = (size_t) (boost::accumulators::max(sx) - boost::accumulators::min(sx) + 1.f);
		size_t sizeY = (size_t) (boost::accumulators::max(sy) - boost::accumulators::min(sy) + 1.f);
		size_t sizeZ = (size_t) (boost::accumulators::max(sz) - boost::accumulators::min(sz) + 1.f);
		denseMatrix dm(sizeX, sizeY, sizeZ);
		int offsetX = (int) boost::accumulators::min(sx);
		int offsetY = (int) boost::accumulators::min(sx);
		int offsetZ = (int) boost::accumulators::min(sx);
		
		cerr << "Max possible dimensions of final shape:" << endl;
		cerr << "x - " << boost::accumulators::min(sx) << ":" << boost::accumulators::max(sx) << endl;
		cerr << "y - " << boost::accumulators::min(sy) << ":" << boost::accumulators::max(sy) << endl;
		cerr << "z - " << boost::accumulators::min(sz) << ":" << boost::accumulators::max(sz) << endl;
		cerr << endl;

		cerr << "Pass 2" << endl;
		// If an input shape is provided, place it into the array.
		if (vm.count("input"))
		{
			for (size_t i=0; i < inshp.numPoints; ++i)
			{
				auto it = inshp.latticePts.block<1,3>(i,0);
				size_t ix = (size_t) (it(0) - offsetX);
				size_t iy = (size_t) (it(1) - offsetY);
				size_t iz = (size_t) (it(2) - offsetZ);
				dm.set(ix, iy, iz, true);
			}
		}

		// Iterate over each fill statement. Expand fill statement.
		for (auto it = vfills.begin(); it != vfills.end(); ++it)
		{
			fillSectorSnowflake(dm,it->rhw,it->c,it->n,it->angles,it->sign);
		}

		// A final pass is needed to calculate the shape center of mass
		cerr << "Pass 3" << endl;
		accumulator_set<double, stats<tag::mean> > mX, mY, mZ;
		for (size_t x=0; x<sizeX; x++)
		{
			for (size_t y=0; y<sizeY; y++)
			{
				for (size_t z=0; z<sizeZ; z++)
				{
					if (dm.get(x,y,z))
					{
						mX((int) x + (int) offsetX);
						mY((int) y + (int) offsetY);
						mZ((int) z + (int) offsetZ);
					}
				}
			}
		}

		// Do direct writeout of shape. No need to invoke shapefile, as this would produce another copy operation.

		auto cWrite = [&](string inval)
		{
			typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
			boost::char_separator<char> sepcom(",");
			tokenizer tk(inval, sepcom);
			for (auto it = tk.begin(); it != tk.end(); ++it)
				out << "\t" << *it;
		};

		cerr << "Writing file" << endl;
		out << vm["title"].as<string>() << endl;
		out << "\t" << dm.numOn << "\t= Number of lattice points" << endl;
		cWrite(vm["a1"].as<string>());
		out << "\t= target vector a1 (in TF)\n";
		cWrite(vm["a2"].as<string>());
		out << "\t= target vector a2 (in TF)\n";
		cWrite(vm["d"].as<string>());
		out << "\t= d_x/d  d_y/d  d_x/d  (normally 1 1 1)\n";

		out << "\t" << boost::accumulators::mean(mX) + offsetX
			<< "\t" << boost::accumulators::mean(mY) + offsetY
			<< "\t" << boost::accumulators::mean(mZ) + offsetZ
			<< "\t= X0(1-3) = location in lattice of target origin" << endl;
		out << "\tNo.\tix\tiy\tiz\tICOMP(x, y, z)" << endl;

		size_t i=1;
		for (size_t x=0; x<sizeX; x++)
		{
			for (size_t y=0; y<sizeY; y++)
			{
				for (size_t z=0; z<sizeZ; z++)
				{
					if (dm.get(x,y,z))
					{
						out << "\t" << i
							<< "\t" << (int) x + (int) offsetX 
							<< "\t" << (int) y + (int) offsetY
							<< "\t" << (int) z + (int) offsetZ
							<< "\t1\t1\t1\n";
						i++;
					}
				}
			}
		}
	}
	catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

