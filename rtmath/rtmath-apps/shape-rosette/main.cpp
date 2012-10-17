/* This is a test program to generate arbitrarily-oriented bullet rosettes.
 * These rosettes may have 3-6 arms and may be placed in any orientation.
 * This program allows for the specification of dipole spacing and rosette 
 * size in physical units and will attempt to align the rosettes as good as 
 * possible given these constraints.
 */

#pragma warning( push )
#pragma warning( disable : 4996 ) // Dumb boost uuid warning
#pragma warning( disable : 4800 ) // forcing non-bool type to true or false
#include <cmath>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <set>

#pragma warning( disable : 4521 ) // Deprecated declaration
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include "../../rtmath/rtmath/PCLlink.h"
#include "../../rtmath/rtmath/VTKlink.h"

#include <boost/program_options.hpp>
#include <boost/shared_array.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/mean.hpp>

//#include "../../rtmath/rtmath/matrixop.h"
//#include "../../rtmath/rtmath/ddscat/rotations.h"
//#include "../../rtmath/rtmath/ddscat/shapefile.h"
//#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/denseMatrix.h"

struct rosetteParams
{
	float armLength;
	float armWidth;
	std::set<size_t> arms;
	float center[3];
	float orientation[3];
};

void fillBox(const float* mins, const float* maxs, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	using namespace std;
	vector<vector<float> > nums;
	nums.resize(3);

	for (size_t i=0;i<3;i++)
	{
		nums[i].reserve( 1 + (size_t) ( maxs[i] -  mins[i])  );
		for (float d = mins[i]; d <= maxs[i]; d++ )
			nums[i].push_back(d);
	}

	size_t sz = nums[0].size() * nums[1].size() * nums[2].size();
	cloud->reserve(cloud->size() + sz);

	for (auto x = nums[0].begin(); x != nums[0].end(); ++x)
		for (auto y = nums[1].begin(); y != nums[1].end(); ++y)
			for (auto z = nums[2].begin(); z != nums[2].end(); ++z)
			{
				pcl::PointXYZ p(*x, *y, *z);
				cloud->push_back(p);
			}
}

void buildArm(size_t arm, float length, float width, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// Arms 0 and 1 are top and bottom.
	// Arms 2 and 3 are left and right.
	// Arms 4 and 5 are front and back.
	// The central arm axis lies along the origin.
	float arm_h = width / 2.0f;
	float mins[3], maxs[3];
	// It's just easier to do a lookup table here
	if (arm <= 1)
	{
		mins[2] = (arm) ? -length : 0;
		maxs[2] = (arm) ? 0 : length;
		mins[0] = mins[1] = -arm_h;
		maxs[0] = maxs[1] = arm_h;
	} else if (arm <= 3) {
		mins[0] = (arm) ? -length : 0;
		maxs[0] = (arm) ? 0 : length;
		mins[1] = mins[2] = -arm_h;
		maxs[1] = maxs[2] = arm_h;
	} else {
		mins[1] = (arm) ? -length : 0;
		maxs[1] = (arm) ? 0 : length;
		mins[0] = mins[2] = -arm_h;
		maxs[0] = maxs[2] = arm_h;
	}

	fillBox(mins, maxs, cloud);
}

void fillRosette(const rosetteParams &p, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	float *mins, float *maxs)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
	// Construct arms
	for (auto arm = p.arms.begin(); arm != p.arms.end(); ++arm)
		buildArm(*arm, p.armLength, p.armWidth, temp);

	// Rotate points
	using namespace boost::accumulators;
		accumulator_set<float, stats<tag::min, tag::max, tag::mean> > sx, sy, sz;
	float nx = p.orientation[0] * boost::math::constants::pi<float>() / 180.0f;
	float ny = p.orientation[1] * boost::math::constants::pi<float>() / 180.0f;
	float nz = p.orientation[2] * boost::math::constants::pi<float>() / 180.0f;
	Eigen::Matrix3f Ref; // Oddly, I can't just have Eigen::Matrix3f Ref = ...
	Ref = Eigen::AngleAxisf(nx, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(ny, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(nz, Eigen::Vector3f::UnitY());
	for (auto pt = temp->begin(); pt != temp->end(); ++pt)
	{
		auto res = Ref * Eigen::Vector3f(pt->x, pt->y, pt->z);
		pt->x = res(0,0);
		pt->y = res(1,0);
		pt->z = res(2,0);
		sx(pt->x);
		sy(pt->y);
		sz(pt->z);
	}
	
	// Translate points to center and add to resulting cloud
	float meanx = boost::accumulators::mean(sx);
	float meany = boost::accumulators::mean(sy);
	float meanz = boost::accumulators::mean(sz);
	float maxx = boost::accumulators::max(sx);
	float maxy = boost::accumulators::max(sy);
	float maxz = boost::accumulators::max(sz);
	float minx = boost::accumulators::min(sx);
	float miny = boost::accumulators::min(sy);
	float minz = boost::accumulators::min(sz);
	cloud->reserve(cloud->size() + temp->size());
	for (auto pt = temp->begin(); pt != temp->end(); ++pt)
	{
		pt->x += p.center[0] - meanx;
		pt->y += p.center[1] - meany;
		pt->z += p.center[2] - meanz;
		cloud->push_back(*pt);
	}
	mins[0] = minx + p.center[0] - meanx;
	mins[1] = miny + p.center[1] - meany;
	mins[2] = minz + p.center[2] - meanz;
	maxs[0] = maxx + p.center[0] - meanx;
	maxs[1] = maxy + p.center[1] - meany;
	maxs[2] = maxz + p.center[2] - meanz;
}

void discretize(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
	const float *mins, const float *maxs, rtmath::denseMatrix &target)
{
	// This function takes a point cloud and translates it into the discrete space defined be a 
	// denseMatrix. The existing denseMatrix will be cleared and resized.
	const size_t sx = (size_t) (maxs[0] - mins[0] + 1);
	const size_t sy = (size_t) (maxs[1] - mins[1] + 1);
	const size_t sz = (size_t) (maxs[2] - mins[2] + 1);

	target.resize(sx,sy,sz);

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud (cloud);

	// Iterate through the possible integer combinations. If an integer point is within 
	// one unit from a point in the cloud, mark it as active.
	for (size_t ix = 0; ix < sx; ix++)
		for (size_t iy = 0; iy < sy; iy++)
			for (size_t iz = 0; iz < sz; iz++)
			{
				// Convert from integer coordinates back to floats in cloud space.
				float fx = ix + mins[0];
				float fy = iy + mins[1];
				float fz = iz + mins[2];

				pcl::PointXYZ searchPoint(fx,fy,fz);

				const int K = 2;
				std::vector<int> pointIdxNKNSearch(K);
				std::vector<float> pointNKNSquaredDistance(K);
				kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
				if (pointNKNSquaredDistance[0] <= 1.0)
				{
					// Point is within the cloud!
					target.set(ix,iy,iz,true);
				}

				/*
				const float radius = 1.01f;
				std::vector<int> pointIdxRadiusSearch;
				std::vector<float> pointRadiusSquaredDistance;
				int nneighbors = 
					kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
				if (nneighbors)
				{
					// Point is within the cloud!
					target.set(ix,iy,iz,true);
				}
				*/
			}
}

int main(int argc, char** argv)
{
	using namespace std;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	float mins[3], maxs[3];
	rosetteParams p;
	p.armLength = 70;
	p.armWidth = 20;
	p.arms.insert(0);
	//p.arms.insert(1);
	p.arms.insert(2);
	p.arms.insert(3);
	p.arms.insert(4);
	p.arms.insert(5);
	p.center[0] = 0;
	p.center[1] = 0;
	p.center[2] = 0;
	p.orientation[0] = 30;
	p.orientation[1] = 15;
	p.orientation[2] = 0;

	fillRosette(p,cloud,mins,maxs);
	rtmath::denseMatrix dm(1,1,1);
	discretize(cloud,mins,maxs,dm);

	cerr << "Writing file" << endl;
	ofstream out("test.shp");
	out <<"title" << endl;
	out << "\t" << dm.numOn << "\t= Number of lattice points" << endl;
	out << "\t1\t0\t0\t= target vector a1 (in TF)\n";
	out << "\t0\t1\t0\t= target vector a2 (in TF)\n";
	out << "\t1\t1\t1\t= d_x/d  d_y/d  d_x/d  (normally 1 1 1)\n";

	out << "\t0\t0\t0\t= X0(1-3) = location in lattice of target origin" << endl;
	out << "\tNo.\tix\tiy\tiz\tICOMP(x, y, z)" << endl;

	size_t i=1;
	const size_t sx = (size_t) (maxs[0] - mins[0] + 1);
	const size_t sy = (size_t) (maxs[1] - mins[1] + 1);
	const size_t sz = (size_t) (maxs[2] - mins[2] + 1);
	for (size_t x=0; x<sx; x++)
	{
		for (size_t y=0; y<sy; y++)
		{
			for (size_t z=0; z<sz; z++)
			{
				if (dm.get(x,y,z))
				{
					out << "\t" << i
						<< "\t" << (int) x 
						<< "\t" << (int) y 
						<< "\t" << (int) z 
						<< "\t1\t1\t1\n";
					i++;
				}
			}
		}
	}
	out.close();
}
