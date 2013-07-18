#include "../rtmath/Stdafx.h"
#pragma warning( disable : 4996 ) // -D_SCL_SECURE_NO_WARNINGS
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <boost/filesystem.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <Ryan_Serialization/serialization.h>

//#include <boost/chrono.hpp>
#include <cmath>
#include "../rtmath/macros.h"
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/error/error.h"

namespace rtmath {
	namespace ddscat {

		shapefile::shapefile()
		{
			_init();
		}

		shapefile::~shapefile() { }

		shapefile::shapefile(const std::string &filename)
		{
			_init();
			read(filename);
			this->filename = filename;
		}

		shapefile::shapefile(std::istream &in)
		{
			_init();
			read(in);
		}

		void shapefile::_init()
		{
			using namespace std;
			numPoints = 0;
			filename = "";
		}

		HASH_t shapefile::hash() const
		{
			std::string res;
			std::ostringstream out;
			write(out);
			res = out.str();
			this->_localhash = HASH(res.c_str(),res.size());
			return this->_localhash;
		}

		void shapefile::read(const std::string &filename)
		{
			using namespace std;
			using namespace boost::interprocess;
			using namespace boost::filesystem;
			// Detect if the input file is compressed
			using namespace Ryan_Serialization;
			std::string cmeth, fname;
			if (!detect_compressed(filename, cmeth, fname))
				throw rtmath::debug::xMissingFile(filename.c_str());

			// Do a direct map into memory. It's faster than stream i/o for reading a large file.
			// Plus, all other operations can be done solely in memory.
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
			this->filename = fname;
			string s(a, fsize);

			// s can contain either compressed or uncompressed input at this point.
			if (cmeth.size())
			{
				boost::iostreams::filtering_istream sin;
				prep_decompression(cmeth, sin);
/// \todo Fix the compressed read stuff. Lots of compile errors
				throw rtmath::debug::xUnimplementedFunction();
				//sin.push(s);
				std::ostringstream buffer;
				/// \todo Check if rdbuf works on a boost iostreams object
				//buffer << sin.rdbuf();
				string suncompressed = buffer.str();
				readString(suncompressed);
			} else {
				readString(s);
			}
		}

		void shapefile::readHeader(std::istream &in)
		{
		}

		void shapefile::readString(const std::string &in)
		{
			// Since istringstream is so slow, I'm dusting off my old atof macros (in 
			// macros.h). These were used when I implemented lbl, and are very fast.
			using namespace std;
			_init();

			// First, do header processing
			//boost::chrono::system_clock::time_point cstart = boost::chrono::system_clock::now();
			size_t pend = 0;
			//boost::chrono::system_clock::time_point cheaderm;
			{
				// Seek to the end of the header, and construct an istringstream for just the header
				size_t pstart = 0;
				for (size_t i=0; i<7; i++)
				{
					pstart = pend;
					pend = in.find_first_of("\n", pend+1);
					size_t posa = 0, posb = pstart;
					Eigen::Array3f *v = nullptr;
					switch (i)
					{
					case 0: // Title line
						desc = string(in.data(),pend);
						break;
					case 1: // Number of dipoles
						{
							// Seek to first nonspace character
							posa = in.find_first_not_of(" \t\n", posb);
							// Find first space after this position
							posb = in.find_first_of(" \t\n", posa);
							size_t len = posb - posa;
							numPoints = rtmath::macros::m_atoi(&(in.data()[posa]),len);
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
							if (2==i) v=&a1;
							if (3==i) v=&a2;
							if (4==i) v=&d;
							if (5==i) v=&x0;
							for (size_t j=0;j<3;j++)
							{
								// Seek to first nonspace character
								posa = in.find_first_not_of(" \t\n,", posb);
								// Find first space after this position
								posb = in.find_first_of(" \t\n,", posa);
								size_t len = posb - posa;
								(*v)(j) = (float) rtmath::macros::m_atof(&(in.data()[posa]),len);
							}
						}
						break;
					}
				}

				latticePts.resize(numPoints,3);
				latticePtsRi.resize(numPoints,3);
				latticePtsStd.resize(numPoints,3);
				latticePtsNorm.resize(numPoints,3);
			}

			using namespace boost::accumulators;
			accumulator_set<float, stats<
				tag::min,
				tag::max, 
				tag::mean> > m_x, m_y, m_z, r_x, r_y, r_z;

			//Eigen::Vector3f crdsm, crdsi; // point location and diel entries
			set<size_t> mediaIds;
			size_t posa = 0, posb = pend+1;
			// Load in the lattice points through iteration and macro.h-based double extraction
			for (size_t i=0; i< numPoints; i++)
			{
				auto crdsm = latticePts.block<1,3>(i,0);
				auto crdsi = latticePtsRi.block<1,3>(i,0);
				for (size_t j=0; j<7; j++)
				{
					// Seek to first nonspace character
					posa = in.find_first_not_of(" \t\n", posb);
					// Find first space after this position
					posb = in.find_first_of(" \t\n", posa);
					size_t len = posb - posa;
					float val;
					val = (float) rtmath::macros::m_atof(&(in.data()[posa]),len);
					if (j==0) continue;
					if (j<=3) crdsm(j-1) = val;
					else crdsi(j-4) = val;
				}

				if (!mediaIds.count((size_t) crdsi(0))) mediaIds.insert((size_t) crdsi(0));
				if (!mediaIds.count((size_t) crdsi(1))) mediaIds.insert((size_t) crdsi(1));
				if (!mediaIds.count((size_t) crdsi(2))) mediaIds.insert((size_t) crdsi(2));

				//latticePts.block<1,3>(i,0) = crdsm;
				//latticePtsRi.block<1,3>(i,0) = crdsi;
			}

			Dielectrics = mediaIds;
			
			// Figure out third lattice vector in target frame
			a3(0) = a1(1)*a2(2)-a1(2)*a2(1);
			a3(1) = a1(2)*a2(0)-a1(0)*a2(2);
			a3(2) = a1(0)*a2(1)-a1(1)*a2(0);

			// Do a second pass and generate the lattice from the lattice points
			// The scaling factors and basis vectors are already in place.
			xd = x0 * d;
			
			//for (auto it = latticePts.begin(); it != latticePts.end(); ++it)
			for (size_t i=0; i< numPoints; i++)
			{
				auto crdsm = latticePts.block<1,3>(i,0);
				//auto crdsi = latticePtsRi.block<1,3>(i,0);
				// Do componentwise multiplication to do scaling
				Eigen::Array3f crd = crdsm.array() * d.transpose();
				auto crdsc = latticePtsStd.block<1,3>(i,0);
				//Eigen::Vector3f -> next line
				crdsc = crd.matrix() - xd.matrix(); // Normalized coordinates!

				// Need to do stat collection here because the midpoint is usually not set correctly!

				r_x(crdsm(0));
				r_y(crdsm(1));
				r_z(crdsm(2));

				m_x(crdsc(0));
				m_y(crdsc(1));
				m_z(crdsc(2));

				// Save in latticePtsStd
				//latticePtsStd.push_back(move(crdsc));
			}

			// Need to renormalize data points. Mean should be at 0, 0, 0 for plotting!
			//for (auto it = latticePtsStd.begin(); it != latticePtsStd.end(); it++)
			for (size_t i=0; i< numPoints; i++)
			{
				auto pt = latticePts.block<1,3>(i,0);
				auto Npt = latticePtsNorm.block<1,3>(i,0);
				//Eigen::Vector3f pt = *it;
				Npt(0) = pt(0) - boost::accumulators::mean(m_x);
				Npt(1) = pt(1) - boost::accumulators::mean(m_y);
				Npt(2) = pt(2) - boost::accumulators::mean(m_z);
				//latticePtsNorm.push_back(move(pt));
			}

			mins(0) = boost::accumulators::min(r_x);
			mins(1) = boost::accumulators::min(r_y);
			mins(2) = boost::accumulators::min(r_z);

			maxs(0) = boost::accumulators::max(r_x);
			maxs(1) = boost::accumulators::max(r_y);
			maxs(2) = boost::accumulators::max(r_z);

			means(0) = boost::accumulators::mean(r_x);
			means(1) = boost::accumulators::mean(r_y);
			means(2) = boost::accumulators::mean(r_z);

			hash();
		}

		void shapefile::write(std::ostream &out) const
		{
			print(out);
		}

		void shapefile::write(const std::string &filename, bool autoCompress) const
		{
			using namespace Ryan_Serialization;
			using namespace std;
			using boost::filesystem::path;
			
			std::string cmeth;
			std::ostringstream outfile;
			if (Ryan_Serialization::detect_compression(filename, cmeth))
				autoCompress = true;
			if (autoCompress)
				Ryan_Serialization::select_compression(filename, cmeth);
			/// \todo Ryan_Serialization::select_compression should also return the compressed 
			/// file name as an optional parameter.
			outfile << filename;
			if (cmeth.size()) outfile << "." << cmeth;
			std::string soutfile = outfile.str();

			ofstream out(soutfile.c_str());
			using namespace boost::iostreams;
			filtering_ostream sout;
			if (cmeth.size())
				prep_compression(cmeth, sout);
			sout.push(out);
			write(sout);
		}

		void shapefile::print(std::ostream &out) const
		{
			using namespace std;
			out << desc << endl;
			out << numPoints << "\t= Number of lattice points" << endl;
			out << a1(0) << "\t" << a1(1) << "\t" << a1(2);
			out << "\t= target vector a1 (in TF)" << endl;
			out << a2(0) << "\t" << a2(1) << "\t" << a2(2);
			out << "\t= target vector a2 (in TF)" << endl;
			out << d(0) << "\t" << d(1) << "\t" << d(2);
			out << "\t= d_x/d  d_y/d  d_x/d  (normally 1 1 1)" << endl;
			out << x0(0) << "\t" << x0(1) << "\t" << x0(2);
			out << "\t= X0(1-3) = location in lattice of target origin" << endl;
			out << "\tNo.\tix\tiy\tiz\tICOMP(x, y, z)" << endl;
			size_t i=1;

			for (size_t j=0; j< numPoints; j++, i++)
			{
				auto it = latticePts.block<1,3>(j,0);
				auto ot = latticePtsRi.block<1,3>(j,0);
				out << "\t" << i << "\t";
				out << (it)(0) << "\t" << (it)(1) << "\t" << (it)(2) << "\t";
				out << (ot)(0) << "\t" << (ot)(1) << "\t" << (ot)(2);
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

