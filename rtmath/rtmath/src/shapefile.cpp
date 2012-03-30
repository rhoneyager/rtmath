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
		_mass = 0;
		_lattice = nullptr;
//		_latticePts.clear();
		_moments.clear();
		_numPoints = 0;
		_filename = "";
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
		throw rtmath::debug::xUnimplementedFunction();
		using namespace std;
		_init();
		// TODO: use boost accumulators to derive
		// 		 the commonly-used statistical quantities
		// First few lines are a header
		// Header ends wit line stating No ix iy iz
		string lin;
		size_t linenum = 0;
		bool done = false;
/*
		std::getline(lin,in);
		_desc = lin;
		std::getline(lin,in);
		_numPoints = boost::lexical_cast<size_t>(lin);

		ostringstream sin;
		double a1[3], a2[3], d[3], x0[3];
		std::getline(lin,in);
		sin.str(lin);
		sin >> a1[0] >> a1[1] >> a1[2];
		std::getline(lin,in);
		sin.str(lin);
		sin >> a2[0] >> a2[1] >> a2[2];
		std::getline(lin,in);
		sin.str(lin);
		sin >> d[0] >> d[1] >> d[2];
		std::getline(lin,in);
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

		// Load in the lattice points
		for (size_t i=0; i < _numPoints; i++)
		{
			std::getline(lin,in);
			sin.str(lin);
			double j, jx, jy, jz, ix, iy, iz;
			sin >> j >> jx >> jy >> jz >> ix >> iy >> iz;
			//coords::cyclic<double> cds(6,jx,jy,jz,ix,iy,iz);
			//_latticePts[i] = cds;
		}

		// Do a second pass and generate the lattice from the lattice points
		// The scaling factors and basis vectors are alreeady in place.

		for (auto it = _latticePts.begin(); it != _latticePts.end(); ++it)
		{
			// First, get matrixops of the lattice vectors
		}*/
	}

	void shapefile::print(std::ostream &out) const
	{
		using namespace std;
		out << _desc << endl;
		out << _numPoints << "\t= Number of lattice points" << endl;
//		_a1->writeTSV(out,false);
		out << "\t= target vector a1 (in TF)" << endl;
//		_a2->writeTSV(out,false);
		out << "\t= target vector a2 (in TF)" << endl;
//		_d->writeTSV(out,false);
		out << "\t= d_x/d  d_y/d  d_x/d  (normally 1 1 1)" << endl;
//		_x0->writeTSV(out,false);
		out << "\t= X0(1-3) = location in lattice of target origin" << endl;
		out << "\tNo.\tix\tiy\tiz\t1\t1\t1" << endl;
/*		for (auto it = _latticePts.begin(); it != _latticePts.end(); ++it)
		{
			out << "\t" << it->first << "\t";
//			it->second.writeTSV(out,false);
			out << endl;
		}*/
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


