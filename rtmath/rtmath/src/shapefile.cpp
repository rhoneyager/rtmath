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
#include <boost/accumulators/statistics/covariance.hpp>
#include <boost/accumulators/statistics/density.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/kurtosis.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/skewness.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/variates/covariate.hpp>
//include <boost/bind.hpp>
//#include <boost/ref.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/make_shared.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
//#include <boost/chrono.hpp>
#include <cmath>
#include "../rtmath/matrixop.h"
#include "../rtmath/error/error.h"
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/macros.h"

namespace rtmath {
	namespace ddscat {

		shapefile::shapefile()
			: _a1(2,1,3), _a2(2,1,3), _a3(2,1,3), _d(2,1,3), _x0(2,1,3), _xd(2,3,1)
		{
			_init();
		}

		shapefile::~shapefile()
		{
		}

		shapefile::shapefile(const std::string &filename)
			: _a1(2,1,3), _a2(2,1,3), _a3(2,1,3), _d(2,1,3), _x0(2,1,3), _xd(2,3,1)
		{
			_init();
			read(filename);
		}

		shapefile::shapefile(std::istream &in)
			: _a1(2,1,3), _a2(2,1,3), _a3(2,1,3), _d(2,1,3), _x0(2,1,3), _xd(2,3,1)
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
			using namespace boost::interprocess;
			using namespace boost::filesystem;

			string fname = filename;
			if (fname == "")
			{
				if (_filename.size())
				{
					fname = _filename;
				} else {
					throw rtmath::debug::xBadInput("Must specify filename");
				}
			}

			if (!exists(path(fname)))
				throw rtmath::debug::xMissingFile(fname.c_str());

			size_t fsize = (size_t) file_size(path(fname)); // bytes

			file_mapping m_file(
				fname.c_str(),
				read_only
				);

			mapped_region region (
				m_file,
				read_only,
				0,
				fsize);

			void* start = region.get_address();
			const char* a = (char*) start;

			_filename = fname;
			string s(a, fsize);
			readString(s);
		}

		void shapefile::read(std::istream &in, size_t length)
		{
			using namespace std;
			if (!length)
			{
				in.seekg(0, ios::end);
				length = (size_t) in.tellg();
				in.seekg(0, ios::beg);
			}

			char* sb = new char[length];
			in.read(sb,length);
			string s(sb,length);
			delete[] sb;

			readString(s);
		}

		void shapefile::readString(const std::string &in)
		{
			// Since istringstream is so slow, I'm dusting off my old atof macros (in 
			// macros.h). These were used when I implemented lbl, and are very fast.
			using namespace std;
			_init();

			size_t length = in.size();

			// First, do header processing
			//boost::chrono::system_clock::time_point cstart = boost::chrono::system_clock::now();
			size_t pend = 0;
			double a1[3], a2[3], a3[3], d[3], x0[3];
			//boost::chrono::system_clock::time_point cheaderm;
			{
				// Seek to the end of the header, and construct an istringstream for just the header
				size_t pstart = 0;
				for (size_t i=0; i<7; i++)
				{
					pstart = pend;
					pend = in.find_first_of("\n", pend+1);
					size_t posa = 0, posb = pstart;
					double *v = nullptr;
					switch (i)
					{
					case 0: // Title line
						_desc = string(in.data(),pend);
						break;
					case 1: // Number of dipoles
						{
							// Seek to first nonspace character
							posa = in.find_first_not_of(" \t\n", posb);
							// Find first space after this position
							posb = in.find_first_of(" \t\n", posa);
							size_t len = posb - posa;
							_numPoints = rtmath::macros::m_atoi(&(in.data()[posa]),len);
						}
						break;
					case 6: // Junk line
					default:
						break;
					case 2: // a1
					case 3: // a2
					case 4: // d
					case 5: // x0
						// These all have the same structure. Read in three doubles, then assign.
						{
							if (2==i) v=a1;
							if (3==i) v=a2;
							if (4==i) v=d;
							if (5==i) v=x0;
							for (size_t j=0;j<3;j++)
							{
								// Seek to first nonspace character
								posa = in.find_first_not_of(" \t\n", posb);
								// Find first space after this position
								posb = in.find_first_of(" \t\n", posa);
								size_t len = posb - posa;
								v[j] = rtmath::macros::m_atof(&(in.data()[posa]),len);
							}
						}
						break;
					}
				}

				
				_latticePts.reserve(_numPoints);
				_latticePtsRi.reserve(_numPoints);
				_latticePtsStd.reserve(_numPoints);
				//cheaderm = boost::chrono::system_clock::now();
				// Note: the static fromDoubleArray constructor returns a shared_ptr<matrixop>,
				// bypassing any compiler return be value/reference difficulties
				_a1.fromDoubleArray(a1);
				_a2.fromDoubleArray(a2);
				_d.fromDoubleArray(d);
				_x0.fromDoubleArray(x0);
			}

			//boost::chrono::system_clock::time_point cheader = boost::chrono::system_clock::now();


			vector<double> valser(7);
			size_t posa = 0, posb = pend+1;
			// Load in the lattice points through iteration and macro.h-based double extraction
			for (size_t i=0; i< _numPoints; i++)
			{
				for (size_t j=0; j<7; j++)
				{
					// Seek to first nonspace character
					posa = in.find_first_not_of(" \t\n", posb);
					// Find first space after this position
					posb = in.find_first_of(" \t\n", posa);
					size_t len = posb - posa;
					double val;
					val = rtmath::macros::m_atof(&(in.data()[posa]),len);
					valser[j] = val;
				}
				// valser[0] is point id, 1-3 are coords, 4-6 are diel entries
				matrixop crdsm(2,1,3), crdsi(2,1,3);
				vector<double>::const_iterator it = valser.begin() + 1;
				crdsm.from<std::vector<double>::const_iterator>(it);
				it += 3;
				crdsi.from<std::vector<double>::const_iterator>(it);

				_latticePts.push_back(move(crdsm));
				_latticePtsRi.push_back(move(crdsi));
				//_latticePts[i] = move(crdsm);
				//_latticePtsRi[i] = move(crdsi);
			}

			//boost::chrono::system_clock::time_point clattice = boost::chrono::system_clock::now();

			// Figure out third lattice vector in target frame
			a3[0] = a1[1]*a2[2]-a1[2]*a2[1];
			a3[1] = a1[2]*a2[0]-a1[0]*a2[2];
			a3[2] = a1[0]*a2[1]-a1[1]*a2[0];
			_a3.fromDoubleArray(a3);

			// Do a second pass and generate the lattice from the lattice points
			// The scaling factors and basis vectors are already in place.
			matrixop xd(2,3,1);
			xd = _x0 % _d;
			_xd = xd;
			
			for (auto it = _latticePts.begin(); it != _latticePts.end(); ++it)
			{
				// First, get matrixops of the lattice vectors
				matrixop crd = *it;
				// Do componentwise multiplication to do scaling
				crd = crd % _d;

				matrixop crdsc = crd - xd; // Normalized coordinates!
				// Save in _latticePtsStd
				_latticePtsStd.push_back(move(crdsc));
			}
			/*
			boost::chrono::system_clock::time_point cnormalized = boost::chrono::system_clock::now();

			boost::chrono::duration<double> dheaderm = cheaderm - cstart;
			boost::chrono::duration<double> dheader = cheader - cstart;
			boost::chrono::duration<double> dlattice = clattice - cheader;
			boost::chrono::duration<double> drenorm = cnormalized - clattice;
			std::cerr << "early header took " << dheaderm.count() << " seconds\n";
			std::cerr << "header took " << dheader.count() << " seconds\n";
			std::cerr << "lattice took " << dlattice.count() << " seconds\n";
			std::cerr << "renorm took " << drenorm.count() << " seconds\n";
			*/
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
			_a1.writeSV("\t",out,false);
			out << "\t= target vector a1 (in TF)" << endl;
			_a2.writeSV("\t",out,false);
			out << "\t= target vector a2 (in TF)" << endl;
			_d.writeSV("\t",out,false);
			out << "\t= d_x/d  d_y/d  d_x/d  (normally 1 1 1)" << endl;
			_x0.writeSV("\t",out,false);
			out << "\t= X0(1-3) = location in lattice of target origin" << endl;
			out << "\tNo.\tix\tiy\tiz\tICOMP(x, y, z)" << endl;
			size_t i=1;
			auto it = _latticePts.begin();
			auto ot = _latticePtsRi.begin();
			for (; it != _latticePts.end(); ++it, ++ot, ++i)
			{
				out << "\t" << i << "\t";
				it->writeSV("\t",out,false);
				ot->writeSV("\t",out,false);
				out << endl;
			}
		}

		boost::shared_ptr<shapefile> shapefile::getPtr() const
		{
			boost::shared_ptr<const shapefile> a = shared_from_this();
			return boost::const_pointer_cast<shapefile>(a);
		}




		shapeFileStats::shapeFileStats(const shapefile &shp)
		{
			_shp = boost::shared_ptr<const shapefile>(new shapefile(shp));
		}

		shapeFileStats::shapeFileStats(const boost::shared_ptr<const shapefile> &shp)
		{
			_shp = shp;
		}

		shapeFileStats::shapeFileStats()
		{
		}

		shapeFileStatsBase::shapeFileStatsBase()
			: mom1(2,3,1), mom2(2,3,1), 
			min(2,3,1), max(2,3,1), sum(2,3,1), skewness(2,3,1), kurtosis(2,3,1), 
			b_min(2,3,1), b_max(2,3,1), b_mean(2,3,1), rot(2,3,3), invrot(2,3,3),
			mominert(2,3,3), _a1(2,1,3), _a2(2,1,3), covariance(2,3,3)
		{
			_N = 0;
			beta = 0;
			theta = 0;
			phi = 0;
			_valid = false;
		}

		shapeFileStatsBase::~shapeFileStatsBase()
		{
		}

		void shapeFileStatsBase::calcStatsRot(double beta, double theta, double phi)
		{
			const double drconv = 2.0*boost::math::constants::pi<double>()/180.0;
			double cb = cos(beta*drconv);
			double ct = cos(theta*drconv);
			double cp = cos(phi*drconv);
			double sb = sin(beta*drconv);
			double st = sin(theta*drconv);
			double sp = sin(phi*drconv);
			// Do right-handed rotation
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

			// Normally, Reff = RyRxRz. But, the rotation is a1,a2-dependent, 
			// which are specified in the file. Apply effective rotation matrix also.
			matrixop Roteff = Ry*Rx*Rz*rot;
			
			shapeFileStatsRotated res(beta,theta,phi);
			
			using namespace boost::accumulators;

			// Figure out potential energy
			// Using moment<1> to rescale value by dividing by the total number of points.
			// I'm currently ignoring the mass term, so am making the ansatz that the whole flake 
			// has unit mass.
			accumulator_set<double, stats<tag::moment<1>> > acc_PE;
				
			for (auto it = _shp->_latticePtsStd.begin(); it != _shp->_latticePtsStd.end(); it++)
			{
				// it->first is the points id. it->second is its matrixop coords (1x3 matrix)
				// Mult by rotaion matrix to get 3x1 rotated matrix
				
				matrixop pt = rot * (it->transpose() - b_mean);
				//vector<double> vpt(3);
				//pt.to<std::vector<double> >(vpt);
				acc_PE(abs(pt.get(2,0,0)));

				// Accumulators are in TF frame? Check against Holly code
			}

			// Are other quantities needed?

			// Export to class matrixops
			res.PE = boost::accumulators::moment<1>(acc_PE);




			// Use std move to insert into set
			rotations.insert(std::move(res));
		}

		void shapeFileStatsBase::calcStatsBase()
		{
			using namespace std;
			// Do calculations of the center of mass, the tensor quantities, and other stuff
			// The functions called here are all indep. of the initial state, as mass, density,
			// volume and everything else have been calculated already.

			// Define the accumulators that we want
			// For each axis, get min, max and the other statistics about the distribution

			// Iterate accumulator as function of radial distance from center of mass

			// Pull in some vars from the shapefile
			_N = _shp->_latticePtsStd.size();

			if (!_N)
			{
				GETOBJKEY();
				throw rtmath::debug::xBadInput("Stats cannot be calculated because the shapefile is not loaded.");
			}

			const matrixop &a1 = _shp->_a1;
			const matrixop &a2 = _shp->_a2;
			const matrixop &a3 = _shp->_a3;
			_a1 = a1;
			_a2 = a2;
			// Figure out the base rotation from a1 = <1,0,0>, a2 = <0,1,0> that 
			// gives the current a1, a2.

			double thetar, betar, phir;
			// Theta is the angle between a1 and xlf
			// From dot product, theta = acos(a1.xlf)
			double dp = a1.get(2,0,0); // need only x component, as xlf is the unit vector in +x
			thetar = acos(dp);

			// From a1 = x_lf*cos(theta) + y_lf*sin(theta)*cos(phi) + z_lf*sin(theta)*sin(phi),
			// can use either y_lf or z_lf components to get phi
			double stheta = sin(thetar);
			if (thetar)
			{
				double acphi = a1.get(2,0,1) / stheta;
				phir = acos(acphi);

				// Finally, a2_x = -sin(theta)cos(beta)
				double cbeta = a2.get(2,0,0) / stheta * -1;
				betar = acos(cbeta);
			} else {
				// theta is zero, so gimbal locking occurs. assume phi = 0.
				phir = 0;
				// must use alternate definition to get beta
				double cosbeta = a2.get(2,0,1);
				betar = acos(cosbeta);
			}

			// thetar, betar, phir are in radians
			// convert to degrees
			{
				double scale = 180.0/(2.0*boost::math::constants::pi<double>());
				beta = betar * scale;
				theta = thetar * scale;
				phi = phir * scale;
			}

			// And figure out the effective rotation matrix of the existing file
			{
				double cb = cos(beta);
				double ct = cos(theta);
				double cp = cos(phi);
				double sb = sin(beta);
				double st = sin(theta);
				double sp = sin(phi);
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

				rot = Ry*Rx*Rz;
				invrot = rot.inverse();
			}

			// Define statistics for max, min, mean, std dev, skewness, kurtosis, moment of inertia
			using namespace boost::accumulators;
			//using namespace boost::accumulators::tag;
			
			// Do two passes to be able to renormalize coordinates
			accumulator_set<double, stats<tag::mean, tag::min, tag::max> > m_x, m_y, m_z;
			for (auto it = _shp->_latticePtsStd.begin(); it != _shp->_latticePtsStd.end(); it++)
			{
				m_x(it->get(2,0,0));
				m_y(it->get(2,0,1));
				m_z(it->get(2,0,2));
			}

			b_min.set(boost::accumulators::min(m_x),2,0,0);
			b_min.set(boost::accumulators::min(m_y),2,1,0);
			b_min.set(boost::accumulators::min(m_z),2,2,0);

			b_max.set(boost::accumulators::max(m_x),2,0,0);
			b_max.set(boost::accumulators::max(m_y),2,1,0);
			b_max.set(boost::accumulators::max(m_z),2,2,0);

			b_mean.set(boost::accumulators::mean(m_x),2,0,0);
			b_mean.set(boost::accumulators::mean(m_y),2,1,0);
			b_mean.set(boost::accumulators::mean(m_z),2,2,0);

			// Tried http://stackoverflow.com/questions/4316716/is-it-possible-to-use-boost-accumulators-with-vectors?rq=1
			// with accumulator_set<vector<double>, ...), but it does not compile on msvc 2010
			accumulator_set<double, stats<
				tag::min,
				tag::max, 
				tag::moment<1>,
				tag::moment<2>,
				tag::sum,
				tag::skewness,
				tag::kurtosis,
				// Covariances are special
				tag::covariance<double, tag::covariate1>,
				tag::covariance<double, tag::covariate2>
				> > acc_x, acc_y, acc_z; //acc(std::vector<double>(3)); //acc_x, acc_y, acc_z;
				
			for (auto it = _shp->_latticePtsStd.begin(); it != _shp->_latticePtsStd.end(); it++)
			{
				// it->first is the points id. it->second is its matrixop coords (1x3 matrix)
				// Mult by rotaion matrix to get 3x1 rotated matrix
				
				matrixop pt = rot * (it->transpose() - b_mean);
				double x = pt.get(2,0,0);
				double y = pt.get(2,1,0);
				double z = pt.get(2,2,0);
				//vector<double> vpt(3);
				//pt.to<std::vector<double> >(vpt);
				acc_x(x, covariate1 = y, covariate2 = z);
				acc_y(y, covariate1 = x, covariate2 = z);
				acc_z(z, covariate1 = x, covariate2 = y);

				// Accumulators are in TF frame? Check against Holly code
			}

			// Are other quantities needed?

			// Export to class matrixops
			min.set(boost::accumulators::min(acc_x),2,0,0);
			min.set(boost::accumulators::min(acc_y),2,1,0);
			min.set(boost::accumulators::min(acc_z),2,2,0);

			max.set(boost::accumulators::max(acc_x),2,0,0);
			max.set(boost::accumulators::max(acc_y),2,1,0);
			max.set(boost::accumulators::max(acc_z),2,2,0);

			sum.set(boost::accumulators::sum(acc_x),2,0,0);
			sum.set(boost::accumulators::sum(acc_y),2,1,0);
			sum.set(boost::accumulators::sum(acc_z),2,2,0);

			skewness.set(boost::accumulators::skewness(acc_x),2,0,0);
			skewness.set(boost::accumulators::skewness(acc_y),2,1,0);
			skewness.set(boost::accumulators::skewness(acc_z),2,2,0);

			kurtosis.set(boost::accumulators::kurtosis(acc_x),2,0,0);
			kurtosis.set(boost::accumulators::kurtosis(acc_y),2,1,0);
			kurtosis.set(boost::accumulators::kurtosis(acc_z),2,2,0);

			mom1.set(boost::accumulators::moment<1>(acc_x),2,0,0);
			mom1.set(boost::accumulators::moment<1>(acc_y),2,1,0);
			mom1.set(boost::accumulators::moment<1>(acc_z),2,2,0);

			mom2.set(boost::accumulators::moment<2>(acc_x),2,0,0);
			mom2.set(boost::accumulators::moment<2>(acc_y),2,1,0);
			mom2.set(boost::accumulators::moment<2>(acc_z),2,2,0);

			//covariance
			covariance.set(_N*boost::accumulators::moment<2>(acc_x),2,0,0);
			//boost::accumulators::covariance(acc_x, covariate1);
			//boost::accumulators::covariance(acc_x, covariate2);
			covariance.set(boost::accumulators::covariance(acc_x, covariate1),2,0,1);
			covariance.set(boost::accumulators::covariance(acc_x, covariate2),2,0,2);
			covariance.set(boost::accumulators::covariance(acc_y, covariate1),2,1,0);
			covariance.set(_N*boost::accumulators::moment<2>(acc_y),2,1,1);
			covariance.set(boost::accumulators::covariance(acc_y, covariate2),2,1,2);
			covariance.set(boost::accumulators::covariance(acc_z, covariate1),2,2,0);
			covariance.set(boost::accumulators::covariance(acc_z, covariate2),2,2,1);
			covariance.set(_N*boost::accumulators::moment<2>(acc_z),2,2,2);

			// Calculate moments of inertia
			{
				double val = 0;
				// All wrong. Need to redo.
				// I_xx
				val = boost::accumulators::moment<2>(acc_y) + boost::accumulators::moment<2>(acc_z);
				mominert.set(val,2,0,0);

				// I_yy
				val = boost::accumulators::moment<2>(acc_x) + boost::accumulators::moment<2>(acc_z);
				mominert.set(val,2,1,1);

				// I_zz
				val = boost::accumulators::moment<2>(acc_x) + boost::accumulators::moment<2>(acc_y);
				mominert.set(val,2,2,2);

				// I_xy and I_yx
				val = -1.0 * covariance.get(2,1,0);
				val /= _N;
				mominert.set(val,2,0,1);
				mominert.set(val,2,1,0);

				// I_xz and I_zx
				val = -1.0 * covariance.get(2,2,0);
				val /= _N;
				mominert.set(val,2,0,2);
				mominert.set(val,2,2,0);

				// I_yz and I_zy
				val = -1.0 * covariance.get(2,2,1);
				val /= _N;
				mominert.set(val,2,2,1);
				mominert.set(val,2,1,2);
			}

			
			_valid = true;
		}

		shapeFileStatsRotated::shapeFileStatsRotated(double beta, double theta, double phi)
		{
			this->beta = beta;
			this->theta = theta;
			this->phi = phi;
			PE = 0;
		}

		shapeFileStatsRotated::shapeFileStatsRotated()
		{
			this->beta = 0;
			this->theta = 0;
			this->phi = 0;
			PE = 0;
		}

		shapeFileStatsRotated::~shapeFileStatsRotated()
		{
		}

		bool shapeFileStatsRotated::operator<(const shapeFileStatsRotated &rhs) const
		{
			if (beta!=rhs.beta) return beta<rhs.beta;
			if (theta!=rhs.theta) return theta<rhs.theta;
			if (phi!=rhs.phi) return phi<rhs.phi;
			return false;
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


//BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapefile)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapeFileStatsBase)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapeFileStats)
