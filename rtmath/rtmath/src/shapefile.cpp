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
#include <boost/accumulators/statistics/kurtosis.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/skewness.hpp>
#include <boost/accumulators/statistics/variance.hpp>
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
			using namespace boost::interprocess;
			using namespace boost::filesystem;
			if (!exists(path(filename)))
				throw rtmath::debug::xMissingFile(filename.c_str());

			size_t fsize = (size_t) file_size(path(filename)); // bytes

			file_mapping m_file(
				filename.c_str(),
				read_only
				);

			mapped_region region (
				m_file,
				read_only,
				0,
				fsize);

			void* start = region.get_address();
			const char* a = (char*) start;

			_filename = filename;
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

			read(s);
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
				_a1 = boost::shared_ptr<matrixop>(matrixop::fromDoubleArray(a1,2,1,3));
				_a2 = boost::shared_ptr<matrixop>(matrixop::fromDoubleArray(a2,2,1,3));
				_d  = boost::shared_ptr<matrixop>(matrixop::fromDoubleArray(d,2,1,3));
				_x0 = boost::shared_ptr<matrixop>(matrixop::fromDoubleArray(x0,2,1,3));
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
			_a3 = boost::shared_ptr<matrixop>(matrixop::fromDoubleArray(a3,2,1,3));

			// Do a second pass and generate the lattice from the lattice points
			// The scaling factors and basis vectors are already in place.
			matrixop xd(2,3,1);
			xd = *_x0 % *_d;
			_xd = boost::make_shared<matrixop>(xd);
			
			for (auto it = _latticePts.begin(); it != _latticePts.end(); ++it)
			{
				// First, get matrixops of the lattice vectors
				matrixop crd = *it;
				// Do componentwise multiplication to do scaling
				crd = crd % *_d;

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
			_a1->writeSV("\t",out,false);
			out << "\t= target vector a1 (in TF)" << endl;
			_a2->writeSV("\t",out,false);
			out << "\t= target vector a2 (in TF)" << endl;
			_d->writeSV("\t",out,false);
			out << "\t= d_x/d  d_y/d  d_x/d  (normally 1 1 1)" << endl;
			_x0->writeSV("\t",out,false);
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




		shapeFileStats::shapeFileStats(const shapefile &shp, double beta, double theta, double phi)
		{
			_shp = shp.getPtr();
			setRot(beta,theta,phi);
		}

		shapeFileStats::shapeFileStats(const boost::shared_ptr<const shapefile> &shp, double beta, double theta, double phi)
		{
			_shp = shp;
			setRot(beta,theta,phi);
		}

		shapeFileStats::shapeFileStats()
		{
		}

		shapeFileStatsBase::shapeFileStatsBase()
			: cm(2,3,3), rot(2,3,3), mom1(2,3,3), mom2(2,3,3)
		{
			rot = matrixop::identity(2,3,3);
			beta = 0;
			theta = 0;
			phi = 0;
			_N = 0;
		}

		shapeFileStatsBase::~shapeFileStatsBase()
		{
		}

		void shapeFileStatsBase::setRot(double beta, double theta, double phi)
		{
			this->beta = beta;
			this->theta = theta;
			this->phi = phi;

			if (beta == 0 && theta == 0 && phi == 0)
			{
				rot = matrixop::identity(2,3,3);
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
			rot = Rtot;
		}

		void shapeFileStatsBase::_calcStats()
		{
			using namespace std;
			// Do calculations of the center of mass, the tensor quantities, and other stuff
			// The functions called here are all indep. of the initial state, as mass, density,
			// volume and everything else have been calculated already.

			// Define the accumulators that we want
			// For each axis, get min, max and the other statistics about the distribution

			// Iterate accumulator as function of radial distance from center of mass

			// 
			_N = _shp->_latticePtsStd.size();
			// Define statistics for max, min, mean, std dev, skewness, kurtosis, moment of inertia
			using namespace boost::accumulators;
			
			// Tried http://stackoverflow.com/questions/4316716/is-it-possible-to-use-boost-accumulators-with-vectors?rq=1
			// with accumulator_set<vector<double>, ...), but it does not compile on msvc 2010
			accumulator_set<double, stats<
				tag::min,
				tag::max, 
				tag::moment<1>,
				tag::moment<2>,
				tag::sum,
				tag::mean, 
				tag::skewness,
				tag::kurtosis,
				tag::variance
				> > acc_x, acc_y, acc_z; //acc(std::vector<double>(3)); //acc_x, acc_y, acc_z;
				
			for (auto it = _shp->_latticePtsStd.begin(); it != _shp->_latticePtsStd.end(); it++)
			{
				// it->first is the points id. it->second is its matrixop coords (3x1 matrix)
				// Mult by rotaion matrix to get 3x1 rotated matrix
				matrixop pt = rot * *it;
				//vector<double> vpt(3);
				//pt.to<std::vector<double> >(vpt);
				acc_x(pt.get(2,0,0));
				acc_y(pt.get(2,1,0));
				acc_z(pt.get(2,2,0));

				// Accumulators are in TF frame? Check against Holly code
			}

			// Are other quantities needed?

			// Export to class matrixops
			mom1.set(boost::accumulators::moment<1>(acc_x),2,0,0);
			mom1.set(boost::accumulators::moment<1>(acc_y),2,1,0);
			mom1.set(boost::accumulators::moment<1>(acc_z),2,2,0);

			mom2.set(boost::accumulators::moment<2>(acc_x),2,0,0);
			mom2.set(boost::accumulators::moment<2>(acc_y),2,1,0);
			mom2.set(boost::accumulators::moment<2>(acc_z),2,2,0);
			GETOBJKEY();
			/*
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


//BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapefile)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapeFileStatsBase)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapeFileStats)
