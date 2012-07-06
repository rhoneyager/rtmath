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
#include <boost/make_shared.hpp>
#include <boost/enable_shared_from_this.hpp>
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
			read(filename);
		}

		shapefile::shapefile(std::istream &in)
		{
			_init();
			read(in);
		}

		void shapefile::_init()
		{
			using namespace std;
			//_lattice = nullptr;
			_latticePts.clear();
			_latticePtsStd.clear();
			_latticePtsRi.clear();
			//_moments.clear();
			_numPoints = 0;
			_filename = "";
			//_I = shared_ptr<matrixop>(new matrixop(2,3,3));
		}

		void shapefile::read(const std::string &filename)
		{
			using namespace std;
			ifstream in(filename.c_str());
			read(in);
			_filename = filename;
		}

		void shapefile::read(std::istream &in)
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

			// Note: the static fromDoubleArray constructor returns a shared_ptr<matrixop>,
			// bypassing any compiler return be value/reference difficulties
			_a1 = boost::shared_ptr<matrixop>(matrixop::fromDoubleArray(a1,2,1,3));
			_a2 = boost::shared_ptr<matrixop>(matrixop::fromDoubleArray(a2,2,1,3));
			_d  = boost::shared_ptr<matrixop>(matrixop::fromDoubleArray(d,2,1,3));
			_x0 = boost::shared_ptr<matrixop>(matrixop::fromDoubleArray(x0,2,1,3));

			std::getline(in,lin); // Skip junk line

			// Load in the lattice points
			for (size_t i=0; i < _numPoints; i++)
			{
				std::getline(in,lin);
				istringstream pin(lin); // MSVC bug with sin not reloading data in loop
				double j, jx, jy, jz, ix, iy, iz;
				pin >> j >> jx >> jy >> jz >> ix >> iy >> iz;
				coords::cyclic<double> cds(6,jx,jy,jz,ix,iy,iz);
				//matrixop crds(2,1,6);
				matrixop crdsm(2,1,3), crdsi(2,1,3);
				cds.get(crdsm,0,3);
				cds.get(crdsi,3,3);
				_latticePts[i+1] = crdsm;
				_latticePtsRi[i+1] = crdsi;
			}

			// Figure out third lattice vector in target frame
			a3[0] = a1[1]*a2[2]-a1[2]*a2[1];
			a3[1] = a1[2]*a2[0]-a1[0]*a2[2];
			a3[2] = a1[0]*a2[1]-a1[1]*a2[0];
			_a3 = boost::shared_ptr<matrixop>(matrixop::fromDoubleArray(a3,2,1,3));

			// Do a second pass and generate the lattice from the lattice points
			// The scaling factors and basis vectors are already in place.
			matrixop xd(2,3,1);
			xd = *_x0 % *_d;
			_xd = boost::make_shared<matrixop>(xd);

			for (auto it = _latticePts.begin(); it != _latticePts.end(); ++it)
			{
				// First, get matrixops of the lattice vectors
				matrixop crd = it->second;
				// Do componentwise multiplication to do scaling
				crd = crd % *_d;

				matrixop crdsc = crd - xd; // Normalized coordinates!
				// Save in _latticePtsStd
				_latticePtsStd[it->first] = crdsc;
			}
		}

		void shapefile::write(std::ostream &out) const
		{
			print(out);
		}

		void shapefile::write(const std::string &filename) const
		{
			using namespace std;
			ofstream out(filename.c_str());
			write(out);
		}

		void shapefile::print(std::ostream &out) const
		{
			using namespace std;
			out << _desc << endl;
			out << _numPoints << "\t= Number of lattice points" << endl;
			_a1->writeSV("\t",out,false);
			out << "\t= target vector a1 (in TF)" << endl;
			_a2->writeSV("\t",out,false);
			out << "\t= target vector a2 (in TF)" << endl;
			_d->writeSV("\t",out,false);
			out << "\t= d_x/d  d_y/d  d_x/d  (normally 1 1 1)" << endl;
			_x0->writeSV("\t",out,false);
			out << "\t= X0(1-3) = location in lattice of target origin" << endl;
			out << "\tNo.\tix\tiy\tiz\t1\t1\t1" << endl;
			for (auto it = _latticePts.begin(); it != _latticePts.end(); ++it)
			{
				out << "\t" << it->first << "\t";
				it->second.writeSV("\t",out,false);
				out << endl;
			}
		}

		boost::shared_ptr<shapefile> shapefile::getPtr() const
		{
			boost::shared_ptr<const shapefile> a = shared_from_this();
			return boost::const_pointer_cast<shapefile>(a);
		}

		shapeFileStats::shapeFileStats(const shapefile &shp, double beta, double theta, double phi)
		{
			_init();
			_shp = shp.getPtr();
			setRot(beta,theta,phi);
		}

		shapeFileStats::shapeFileStats(const boost::shared_ptr<const shapefile> &shp, double beta, double theta, double phi)
		{
			_init();
			_shp = shp;
			setRot(beta,theta,phi);
		}

		void shapeFileStats::_init()
		{
			//_shp = nullptr;
			_rot = boost::make_shared<matrixop>(matrixop::identity(2,3,3));
			_beta = 0;
			_theta = 0;
			_phi = 0;
		}

		void shapeFileStats::setRot(double beta, double theta, double phi)
		{
			_beta = beta;
			_theta = theta;
			_phi = phi;

			if (beta == 0 && theta == 0 && phi == 0)
			{
				_rot = boost::make_shared<matrixop>(matrixop::identity(2,3,3));
				return;
			}

			const double drconv = 2.0*boost::math::constants::pi<double>()/180.0;
			double cb = cos(beta*drconv);
			double ct = cos(theta*drconv);
			double cp = cos(phi*drconv);
			double sb = sin(beta*drconv);
			double st = sin(theta*drconv);
			double sp = sin(phi*drconv);
			// Do left-handed rotation
			// It's just easier to express the overall rotation as the multiplication of
			// the component Gimbal matrices.
			matrixop Rx(2,3,3), Ry(2,3,3), Rz(2,3,3);

			Rx.set(1,2,0,0);
			Rx.set(cp,2,1,1);
			Rx.set(cp,2,2,2);
			Rx.set(sp,2,2,1);
			Rx.set(-sp,2,1,2);

			Ry.set(cb,2,0,0);
			Ry.set(1 ,2,1,1);
			Ry.set(cb,2,2,2);
			Ry.set(sb,2,0,2);
			Ry.set(-sb,2,2,0);

			Rz.set(ct,2,0,0);
			Rz.set(ct,2,1,1);
			Rz.set(1,2,2,2);
			Rz.set(st,2,1,0);
			Rz.set(-st,2,0,1);

			matrixop Rtot = Rz*Rx*Ry;
			_rot = boost::make_shared<matrixop>(Rtot);
		}

		void shapeFileStats::_calcOtherStats()
		{
			// Do calculations of the center of mass, the tensor quantities, and other stuff
			// The functions called here are all indep. of the initial state, as mass, density,
			// volume and everything else have been calculated already.

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
	ob.read(stream);
	return stream;
}


