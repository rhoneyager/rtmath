#include "../rtmath/Stdafx.h"
#include <pcl/point_types.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/make_shared.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/accumulators/statistics/mean.hpp>
//#include <boost/chrono.hpp>
#include <cmath>
#include "../rtmath/matrixop.h"
#include "../rtmath/error/error.h"
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/macros.h"
#include "../rtmath/Garrett/pclstuff.h"

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
			_filename = filename;
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
			set<size_t> mediaIds;
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
				crdsm.set(valser[1],2,0,0);
				crdsm.set(valser[2],2,0,1);
				crdsm.set(valser[3],2,0,2);
				crdsi.set(valser[4],2,0,0);
				crdsi.set(valser[5],2,0,1);
				crdsi.set(valser[6],2,0,2);
				//GETOBJKEY();
				/* // if only.....
				vector<double>::const_iterator it = valser.begin() + 1;
				crdsm.from<std::vector<double>::const_iterator>(it);
				it += 3;
				crdsi.from<std::vector<double>::const_iterator>(it);
				*/
				if (mediaIds.count(valser[4]) == 0) mediaIds.insert(valser[4]);

				_latticePts.push_back(move(crdsm));
				_latticePtsRi.push_back(move(crdsi));
				//_latticePts[i] = move(crdsm);
				//_latticePtsRi[i] = move(crdsi);
			}

			_Dielectrics = mediaIds;
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
			
			using namespace boost::accumulators;
			accumulator_set<double, stats<tag::mean> > m_x, m_y, m_z;

			for (auto it = _latticePts.begin(); it != _latticePts.end(); ++it)
			{
				// First, get matrixops of the lattice vectors
				matrixop crd = *it;
				// Do componentwise multiplication to do scaling
				crd = crd % _d;

				matrixop crdsc = crd - xd; // Normalized coordinates!
				// Need to do stat collection here because the midpoint is usually not set correctly!

				m_x(crdsc.get(2,0,0));
				m_y(crdsc.get(2,0,1));
				m_z(crdsc.get(2,0,2));

				// Save in _latticePtsStd
				_latticePtsStd.push_back(move(crdsc));
			}


			// And also construct the basic pointContainer object that holds the points (for faster 
			// hull and meshing operations)

			_pclObj = boost::shared_ptr<rtmath::Garrett::pointContainer>
				(new rtmath::Garrett::pointContainer);

			_pclObj->cloud->reserve(_latticePtsStd.size());
			
			// Need to renormalize data points in point cloud. Mean should be at 0, 0, 0 for plotting!
			for (auto it = _latticePtsStd.begin(); it != _latticePtsStd.end(); it++)
			{
				const double x = it->get(2,0,0) - boost::accumulators::mean(m_x);
				const double y = it->get(2,0,1) - boost::accumulators::mean(m_y);
				const double z = it->get(2,0,2) - boost::accumulators::mean(m_z);
				_pclObj->cloud->push_back(pcl::PointXYZ(x,y,z));
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

