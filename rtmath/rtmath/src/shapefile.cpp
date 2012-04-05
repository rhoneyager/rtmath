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
#include <boost/math/constants/constants.hpp>
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
		sin.clear();
		sin.str(lin);
		sin >> a1[0] >> a1[1] >> a1[2];
		std::getline(in,lin);
		sin.clear();
		sin.str(lin);
		sin >> a2[0] >> a2[1] >> a2[2];
		std::getline(in,lin);
		sin.clear();
		sin.str(lin);
		sin >> d[0] >> d[1] >> d[2];
		std::getline(in,lin);
		sin.clear();
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
			_latticePts[i+1] = cds;
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

		for (auto it = _latticePts.begin(); it != _latticePts.end(); ++it)
		{
			// First, get matrixops of the lattice vectors
			matrixop crd(2,3,1);
			it->second.get(crd,0,3); // Drop the last 3 crds (they don't matter here)
			// Do componentwise multiplication to do scaling
			crd = crd % scale;

			_mass++;
			matrixop crdsc = crd - xd; // Normalized coordinates!

			coords::cyclic<double> ccc(crdsc);
			// Save in _latticePtsStd
			_latticePtsStd[it->first] = ccc;
		}
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
		out << "\tNo.\tix\tiy\tiz\t1\t1\t1" << endl;
		for (auto it = _latticePts.begin(); it != _latticePts.end(); ++it)
		{
			out << "\t" << it->first << "\t";
			it->second.writeTSV(out,false);
			out << endl;
		}
	}

	std::shared_ptr<shapefile> shapefile::getPtr() const
	{
		std::shared_ptr<const shapefile> a = shared_from_this();
		return std::const_pointer_cast<shapefile>(a);
	}

	shapeFileStats::shapeFileStats(const shapefile &shp)
	{
		_init();
		_shp = shp.getPtr();
	}

	shapeFileStats::shapeFileStats(const std::shared_ptr<const shapefile> &shp)
	{
		_init();
		_shp = shp;
	}

	void shapeFileStats::_init()
	{
		_d = 0;
		_V = 0;
		_N = 0;
		_reff = 0;
		_T = 263; // TODO: find an appropriate value for this
		_mass = 0;
		_density = 0;
		_valid = false;
		_shp = nullptr;
	}

	void shapeFileStats::_calcDensities()
	{
		// The density of water and ice is expected to change with temperature.
		// Knowing volume, this will enable a determination of mass

		// TODO: implement this later.
	}

	void shapeFileStats::_calcDensity()
	{
		// Given the densities of the known constituents, calculate the overall particle
		// mass and its density

		double uV = _V / (double) _N; // unit volume
		double mass = 0;
		for (auto it = _shp->_latticePtsStd.begin(); it != _shp->_latticePtsStd.end(); ++it)
		{
			size_t material = (size_t) it->second.get(3);
			double den = 1.0;
			if (_densities.count(material)) // _calcDensities not really implemented yet...
				den = _densities.at(material);
			double um = den * uV;
			mass += um;
		}
		_mass = mass;
		_density = mass / _V;
	}

	void shapeFileStats::_calcFromD()
	{
		// Do calculations from interdipole spacing.

		// First, find volume
		// Assume eash dipole is a cube. Take number of dipoles and 
		// multiply by interdipole distance.
		double Vd = pow(_d,3.0);
		_V = ((double) _N) * Vd;

		// Get effective radius
		// Take volume, and treat the object as a sphere. Then, 
		// determine the effective radius, used in ddscat calculations.
		double pi = boost::math::constants::pi<double>();
		_reff = pow(3.*_V/(4.*pi),1./3.);

		// Calculate mass (kg). Assume knowledge of all densities.
		// This changes based on temperature
		_calcDensities();

		// Calculate overall density and particle mass
		_calcDensity();

		// Calculate other quantities, like center of mass and inertia tensor
		_calcOtherStats();
	}

	void shapeFileStats::_calcFromV()
	{
		// With a known volume, the interdipole spacing can be calculated
		double Vd = _V / ((double) _N);
		_d = pow(Vd,1./3.);

		// Get effective radius
		double pi = boost::math::constants::pi<double>();
		_reff = pow(3.*_V/(4.*pi),1./3.);

		_calcDensities();
		_calcDensity();
		_calcOtherStats();
	}

	void shapeFileStats::_calcFromReff()
	{
		// Get volume
		double pi = boost::math::constants::pi<double>();
		_V = 4./3. * pi * pow(_reff,3.0);
		double Vd = _V / ((double) _N);
		_d = pow(Vd,1./3.);

		_calcDensities();
		_calcDensity();
		_calcOtherStats();
	}

	void shapeFileStats::_calcFromMass()
	{
		// Given temp, can figure out densities of crystal components.
		// Can also get overall crystal density
		// From this, can get volume
		// Then, can calculate effective radius
		// And can get interdipole spacing
	}

	void shapeFileStats::_calcOtherStats()
	{
		// Do calculations of the center of mass, the tensor quantities, and other stuff
		// The functions called here are all indep. of the initial state, as mass, density,
		// volume and everything else have been calculated already.
		_valid = true;

		// Define the accumulators that we want
		// For each axis, get min, max and the other statistics about the distribution

		// Iterate accumulator as function of radial distance from center of mass

		// 
		/*
				// Define statistics for max, min, mean, std dev, skewness, kurtosis, moment of inertia
		using namespace boost::accumulators;

		accumulator_set<double, stats<
			tag::(min),
			tag::(max), 
			tag::sum,
			tag::mean, 
			tag::moment<2>,
			tag::skewness,
			tag::kurtosis
		> > acc_x, acc_y, acc_z;

		accumulator_set<double, stats<
			tag::sum
		> > iner_xx, iner_yy, iner_zz, iner_xy, iner_xz, iner_yz;
		*/
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


