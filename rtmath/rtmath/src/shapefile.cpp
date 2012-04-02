#include "../rtmath/Stdafx.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <complex>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <cmath>
#include "../rtmath/matrixop.h"
#include "../rtmath/error/error.h"
#include "../rtmath/ddscat/shapefile.h"

namespace rtmath {
	namespace ddscat {

	shapefile::shapefile()
	{
		_init();
	}

	shapefile::~shapefile()
	{
	}

	shapefile::shapefile(const std::string &filename)
	{
		_init();
		loadFile(filename);
	}

	shapefile::shapefile(std::istream &in)
	{
		_init();
		loadFile(in);
	}

	void shapefile::_init()
	{
		using namespace std;
		_mass = 0;
		_lattice = nullptr;
		_latticePts.clear();
		_latticePtsStd.clear();
		_moments.clear();
		_numPoints = 0;
		_filename = "";
		_I = shared_ptr<matrixop>(new matrixop(2,3,3));
	}

	void shapefile::loadFile(const std::string &filename)
	{
		using namespace std;
		ifstream in(filename.c_str());
		loadFile(in);
		_filename = filename;
	}

	void shapefile::loadFile(std::istream &in)
	{
		//throw rtmath::debug::xUnimplementedFunction();
		using namespace std;
		_init();

		string lin;
		size_t linenum = 0;
		bool done = false;

		std::getline(in,lin);
		_desc = lin;
		std::getline(in,lin);
		istringstream sin;
		sin.str(lin);
		sin >> _numPoints;

		double a1[3], a2[3], a3[3], d[3], x0[3];
		std::getline(in,lin);
		sin.str(lin);
		sin >> a1[0] >> a1[1] >> a1[2];
		std::getline(in,lin);
		sin.str(lin);
		sin >> a2[0] >> a2[1] >> a2[2];
		std::getline(in,lin);
		sin.str(lin);
		sin >> d[0] >> d[1] >> d[2];
		std::getline(in,lin);
		sin.str(lin);
		sin >> x0[0] >> x0[1] >> x0[2];

		_a1 = std::shared_ptr<const coords::cyclic<double> >
			(new coords::cyclic<double>(3,a1[0],a1[1],a1[2]));
		_a2 = std::shared_ptr<const coords::cyclic<double> >
			(new coords::cyclic<double>(3,a2[0],a2[1],a2[2]));
		_d  = std::shared_ptr<const coords::cyclic<double> >
			(new coords::cyclic<double>(3,d[0],d[1],d[2]));
		_x0 = std::shared_ptr<const coords::cyclic<double> >
			(new coords::cyclic<double>(3,x0[0],x0[1],x0[2]));

		std::getline(in,lin); // Skip junk line

		// Load in the lattice points
		for (size_t i=0; i < _numPoints; i++)
		{
			std::getline(in,lin);
			istringstream pin(lin); // MSVC bug with sin not reloading data in loop
			double j, jx, jy, jz, ix, iy, iz;
			pin >> j >> jx >> jy >> jz >> ix >> iy >> iz;
			coords::cyclic<double> cds(6,jx,jy,jz,ix,iy,iz);
			_latticePts[i] = cds;
		}

		// Figure out third lattice vector in target frame
		a3[0] = a1[1]*a2[2]-a1[2]*a2[1];
		a3[1] = a1[2]*a2[0]-a1[0]*a2[2];
		a3[2] = a1[0]*a2[1]-a1[1]*a2[0];
		_a3 = std::shared_ptr<const coords::cyclic<double> >
			(new coords::cyclic<double>(3,a3[0],a3[1],a3[2]));

		// Do a second pass and generate the lattice from the lattice points
		// The scaling factors and basis vectors are already in place.
		matrixop scale(2,3,1);
		_d->get(scale);
		matrixop xd(2,3,1);
		_x0->get(xd);
		xd = xd % scale;
		_xd = std::shared_ptr<const coords::cyclic<double> >
			(new coords::cyclic<double> (xd));

		// Define statistics for max, min, mean, std dev, skewness, kurtosis, moment of inertia
		using namespace boost::accumulators;
		/*
		accumulator_set<double, stats<
			tag::(min),
			tag::(max), 
			tag::sum,
			tag::mean, 
			tag::moment<2>,
			tag::skewness,
			tag::kurtosis
		> > acc_x, acc_y, acc_z;
		*/

		accumulator_set<double, stats<
			tag::sum
		> > iner_xx, iner_yy, iner_zz, iner_xy, iner_xz, iner_yz;

		for (auto it = _latticePts.begin(); it != _latticePts.end(); ++it)
		{
			// First, get matrixops of the lattice vectors
			matrixop crd(2,3,1);
			it->second.get(crd,3); // Drop the last 3 crds (they don't matter here)
			// Do componentwise multiplication to do scaling
			crd = crd % scale;

			_mass++;
			matrixop crdsc = crd - xd; // Normalized coordinates!

			coords::cyclic<double> ccc(crdsc);
			// Save in _latticePtsStd
			_latticePtsStd[it->first] = ccc;
			double x = ccc.get(0), y = ccc.get(1), z = ccc.get(2);

			// Add to accumulators
			// I split the coordinates to make it less work than for extending
			// coords of matrixop...
			/*
			acc_x(x);
			acc_y(y);
			acc_z(z);
			*/

			iner_xx(y*y+z*z);
			iner_yy(x*x+z*z);
			iner_zz(x*x+y*y);
			iner_xy(-x*y);
			iner_xz(-x*z);
			iner_yz(-y*z);
		}


		_I->set(sum(iner_xx),2,0,0);
		_I->set(sum(iner_yy),2,1,1);
		_I->set(sum(iner_zz),2,2,2);
		_I->set(sum(iner_xy),2,0,1);
		_I->set(sum(iner_xy),2,1,0);
		_I->set(sum(iner_xz),2,0,2);
		_I->set(sum(iner_xz),2,2,0);
		_I->set(sum(iner_yz),2,1,2);
		_I->set(sum(iner_yz),2,2,1);
	}

	void shapefile::print(std::ostream &out) const
	{
		using namespace std;
		out << _desc << endl;
		out << _numPoints << "\t= Number of lattice points" << endl;
		_a1->writeTSV(out,false);
		out << "\t= target vector a1 (in TF)" << endl;
		_a2->writeTSV(out,false);
		out << "\t= target vector a2 (in TF)" << endl;
		_d->writeTSV(out,false);
		out << "\t= d_x/d  d_y/d  d_x/d  (normally 1 1 1)" << endl;
		_x0->writeTSV(out,false);
		out << "\t= X0(1-3) = location in lattice of target origin" << endl;
		out << "Tensor of moment of inertia (dimensionless):\n";
		_I->print(out);
		out << "\tNo.\tix\tiy\tiz\t1\t1\t1" << endl;
		for (auto it = _latticePts.begin(); it != _latticePts.end(); ++it)
		{
			out << "\t" << it->first << "\t";
			it->second.writeTSV(out,false);
			out << endl;
		}
	}

	}
}

std::ostream & operator<<(std::ostream &stream, const rtmath::ddscat::shapefile &ob)
{
	ob.print(stream);
	return stream;
}

std::istream & operator>>(std::istream &stream, rtmath::ddscat::shapefile &ob)
{
	ob.loadFile(stream);
	return stream;
}


