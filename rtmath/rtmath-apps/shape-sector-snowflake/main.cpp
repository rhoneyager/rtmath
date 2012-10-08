/* This program produces shapefiles based on rectangular prisms.
 * Multiple prisms may be selected, and 'negative' regions may 
 * be excised. This allows for the generation of a few rather 
 * useful test cases.
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

//#pragma warning( disable : 4521 ) // Deprecated declaration
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
#include <Eigen/Dense>

#include <boost/program_options.hpp>
#include <boost/shared_array.hpp>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/matrixop.h"
#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/denseMatrix.h"

void rangeSectorSnowflake(const double rhw[3], const double c[3], 
	double mins[3], double maxs[3])
{
	using namespace std;
	// This can be narrowed, but it's irrelevent for now
	double rMax = max (max( rhw[0]+1, rhw[1]+1), rhw[2]+1);
	for (size_t i=0; i<3; i++)
	{
		mins[i] = c[i] - rMax;
		maxs[i] = c[i] + rMax;
	}
}

void fillSectorSnowflake(rtmath::denseMatrix &dm, const double rhw[3], const double c[3], 
	const double n[3], const std::set<double> &angles, bool sign)
{
	using namespace std;
	using namespace rtmath;
	// Draw half-ellipses specified by the inputs to make sector snowflake shapes
	// For speec, use the PCL. Create a uniform grid, then filter based on constraints.
	// Constraints include the angle (plane normal) and the bounding sphere.

	// Build the base arm shape
	double mins[3], maxs[3];
	rangeSectorSnowflake(rhw, c, mins, maxs);
	double sizeX = maxs[0] - mins[0] + 1.0;
	double sizeY = maxs[1] - mins[1] + 1.0;
	double sizeZ = maxs[2] - mins[2] + 1.0;

	Eigen::ArrayXd v(sizeX*sizeY*sizeZ);
	size_t sV = 0;

	//rtmath::denseMatrix mask((size_t) sizeX,(size_t) sizeY,(size_t) sizeZ);
	for (double x = mins[0]; x <=maxs[0]; x++)
	{
		for (double y = mins[1]; y <=maxs[1]; y++)
		{
			for (double z = mins[2]; z <=maxs[2]; z++)
			{
				if (0.25 >= pow((x-c[0])/rhw[0],2.0) + pow((y-c[1])/rhw[1],2.0) + pow((z-c[2])/rhw[2],2.0))
				{
					if (x-c[0] >= 0)
					{
						v(sV) = x-c[0];
						v(sV+1) = y-c[1];
						v(sV+2) = z-c[2];
						sV += 3;
						//v << x-mins[0], y-mins[1], z-mins[2];
						//mask.set(x-mins[0],y-mins[1],z-mins[2],true);
						if (x-mins[0] > 200 || y-mins[1] > 200 || z-mins[2] > 200)
						{
							cerr << "Error\n";
						}
						dm.set(x-mins[0],y-mins[1],z-mins[2],true);
					}
				}
			}
		}
	}
	/*
	double nx = n[0] * boost::math::constants::pi<double>() / 180.0;
	double ny = n[1] * boost::math::constants::pi<double>() / 180.0;
	double nz = n[2] * boost::math::constants::pi<double>() / 180.0;

	// Now, take the base arm and duplicate it into the appropriate locations in dm
	for (auto it = angles.begin(); it != angles.end(); ++it)
	{
		// rhw are the semimajor axes lengths
		// c is the center
		// n is the collection of rotations relative to the +x direction
		// each angle is the angle beta for the shape rotation
		double na = *it * boost::math::constants::pi<double>() / 180.0;

		// Construct corresponding rotation and inverse rotation matrices
		Eigen::Matrix3d Ref, Rinv; // Ref should be YXZ?
		Ref = Eigen::AngleAxisd(nx, Eigen::Vector3d::UnitZ())
			* Eigen::AngleAxisd(ny, Eigen::Vector3d::UnitX())
			* Eigen::AngleAxisd(nz+na, Eigen::Vector3d::UnitY());

		Rinv = Ref.inverse();
		cout << Rinv;
		cout << endl;

		for (size_t i = 0; i< sV; i += 3)
		{
			auto res = Rinv * v.segment(i,3).matrix();
			cout << res << endl;
			// Take the translated point and store in dm
			size_t sx = (size_t) (res(0,0) - mins[0]+1);
			size_t sy = (size_t) (res(1,0) - mins[1]+1);
			size_t sz = (size_t) (res(2,0) - mins[2]+1);
			if (sx > 200 || sy > 200 || sz > 200)
			{
				cerr << "Error\n";
			}
			dm.set(sx,sy,sz,sign);
		}
	}
	*/
}

struct fillSet
{
	double rhw[3], c[3], n[3];
	std::set<double> angles;
	bool sign;
};

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-shape-sector-snowflake\n\n";
		rtmath::debug::appEntry(argc, argv);

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("fill", -1);

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("input,i", po::value<string>(), "If an input file is specified, then the file is loaded and edited. Its header is ignored.")
			("output,o", po::value<string>(), "This is the output filename. If unspecified, write to stdout.")
			("title", po::value<string>()->default_value("sector-snowflake-generated shape"), "The description enclosed in the shape file")
			("a1", po::value<string>()->default_value("1,0,0"), "The a1 vector in csv form")
			("a2", po::value<string>()->default_value("0,1,0"), "The a2 vector in csv form")
			("d", po::value<string>()->default_value("1,1,1"), "Dipole scaling factor")
			("fill,f", po::value< vector<string> >(), "Specify a region to be filled. Region selected as a colon and comma-separated "
			"list of r:h:w,narms,cx:cy:cz,nx:ny:nz,sign. "
			"rhw are the ellipsoid semimajor axes values. "
			"narms (defaults to six) is the number of half-ellipsoids to draw (evenly-spaced through 360 degrees). "
			"c{xyz} is the center of the flake in dipole coords. "
			"n{xyz} (optional) is the rotation in degrees of the shape using gimbal matrices. "
			"Sign (optional) determines if we are adding or removing area.");

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

		if (vm.count("fill"))
			vector<string> fills = vm["fill"].as< vector<string> >();
		vector<fillSet> vfills;
		fillSet a;
		a.rhw[0] = 60;
		a.rhw[1] = 10;
		a.rhw[2] = 20;
		a.c[0] = 0;
		a.c[1] = 0;
		a.c[2] = 0;
		a.sign = true;
		a.n[0] = 0;
		a.n[1] = 0;
		a.n[2] = 0;
		a.angles.insert(0);
		//a.angles.insert(180);
		vfills.push_back(move(a));

		/* "list of r:h:w,narms,cx:cy:cz,nx:ny:nz,sign. "
			"rhw are the ellipsoid semimajor axes values. "
			"narms (defaults to six) is the number of half-ellipsoids to draw (evenly-spaced through 360 degrees). "
			"c{xyz} is the center of the flake in dipole coords. "
			"n{xyz} (optional) is the rotation in degrees of the shape using gimbal matrices. "
			"Sign (optional) determines if we are adding or removing area."
			

		// A lambda determining how the point ranges are split
		auto fSplit = [](string inval, double rhw[3], bool &hasC, double c[3], bool &hasN, double n[3], size_t &narms, int mins[3], int maxs[3], int &sign)
		{
			sign = 1;
			typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
			boost::char_separator<char> sepcom(",");
			boost::char_separator<char> sepcol(":");
			tokenizer tp(inval, sepcom);
			size_t selector = 0;
			for (auto it = tp.begin(); it != tp.end(); ++it, ++selector)
			{
				vector<double> v;
				tokenizer tc(*it, sepcol);
				for (auto ot = tc.begin(); ot != tc.end(); ++ot)
					v.push_back(boost::lexical_cast<double>(*ot));

				if (selector == 0)
				{
					rhw[0] = v[0];
					rhw[1] = v[1];
					rhw[2] = v[2];
					continue;
				}
				if (selector == 1)
				{
					if (v.size())
					{
					} else {
						narms = 6;
					}
				}

				if (selector < 3)
				{
					tokenizer tc(*it, sepcol);
					auto p = tc.begin();
					if (p == tc.end()) throw;
					mins[selector] = boost::lexical_cast<int>(*p);
					++p;
					if (p == tc.end())
						maxs[selector] = mins[selector];
					else
						maxs[selector] = boost::lexical_cast<int>(*p);
				} else {
					sign = boost::lexical_cast<int>(*it);
					return;
				}
			}
		};*/

		// Do three passes. First pass allows for dimensioning and memory allocation.
		// Second pass fills in the shape into memory. Dense memory layout is selected, since we are dealing with rectangles.
		// Third pass writes the dipoles to disk
		using namespace boost::accumulators;
		accumulator_set<double, stats<tag::min, tag::max> > sx, sy, sz;
		accumulator_set<double, stats<tag::sum> > svolmax;
		cerr << "Pass 1" << endl;
		for (auto it = vfills.begin(); it != vfills.end(); ++it)
		{
			double imins[3], imaxs[3];
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
			sx(inStats.b_min.get(2,0,0));
			sx(inStats.b_max.get(2,0,0));
			sy(inStats.b_min.get(2,1,0));
			sy(inStats.b_max.get(2,1,0));
			sz(inStats.b_min.get(2,2,0));
			sz(inStats.b_max.get(2,2,0));
		}

		// svolmax now has the max amount of memory required for processing.
		size_t sizeX = boost::accumulators::max(sx) - boost::accumulators::min(sx) + 1;
		size_t sizeY = boost::accumulators::max(sy) - boost::accumulators::min(sy) + 1;
		size_t sizeZ = boost::accumulators::max(sz) - boost::accumulators::min(sz) + 1;
		denseMatrix dm(sizeX, sizeY, sizeZ);
		size_t offsetX = boost::accumulators::min(sx);
		size_t offsetY = boost::accumulators::min(sx);
		size_t offsetZ = boost::accumulators::min(sx);
		
		cerr << "Max possible dimensions of final shape:" << endl;
		cerr << "x - " << boost::accumulators::min(sx) << ":" << boost::accumulators::max(sx) << endl;
		cerr << "y - " << boost::accumulators::min(sy) << ":" << boost::accumulators::max(sy) << endl;
		cerr << "z - " << boost::accumulators::min(sz) << ":" << boost::accumulators::max(sz) << endl;
		cerr << endl;

		cerr << "Pass 2" << endl;
		// If an input shape is provided, place it into the array.
		if (vm.count("input"))
		{
			for (auto it = inshp._latticePts.begin(); it != inshp._latticePts.end(); ++it)
			{
				size_t ix = (size_t) it->get(2,0,0) - offsetX;
				size_t iy = (size_t) it->get(2,0,1) - offsetY;
				size_t iz = (size_t) it->get(2,0,2) - offsetZ;
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
		for (int x = boost::accumulators::min(sx); x <= boost::accumulators::max(sx); x++)
		{
			for (int y = boost::accumulators::min(sy); y <= boost::accumulators::max(sy); y++)
			{
				for (int z = boost::accumulators::min(sz); z <= boost::accumulators::max(sz); z++)
				{
					mX((int) x - (int) offsetX );
					mY((int) y - (int) offsetY);
					mZ((int) z - (int) offsetZ);
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
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		cerr << endl;
		return 1;
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

