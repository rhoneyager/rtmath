#include "Stdafx-ddscat.h"
#pragma warning( disable : 4996 ) // -D_SCL_SECURE_NO_WARNINGS
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <cmath>
//#include <boost/chrono.hpp>
#include <boost/filesystem.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/lexical_cast.hpp>
#include <thread>
#include <mutex>

#include <Ryan_Serialization/serialization.h>
#include "../rtmath/macros.h"
#include "../rtmath/hash.h"
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/ddscat/shapestats.h"
#include "../rtmath/ddscat/hulls.h" // Hulls and VTK functions
#include "../rtmath/error/debug.h"
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
			if (_localhash.lower) return _localhash;
			return rehash();
		}

		HASH_t shapefile::rehash() const
		{
			if (numPoints)
			{
				std::string res;
				std::ostringstream out;
				write(out);
				res = out.str();
				this->_localhash = HASH(res.c_str(),(int) res.size());
			}
			return this->_localhash;
		}

		boost::shared_ptr<shapefile> shapefile::loadHash(
				const HASH_t &hash)
		{
			return loadHash(boost::lexical_cast<std::string>(hash.lower));
		}

		boost::shared_ptr<shapefile> shapefile::loadHash(
			const std::string &hash)
		{
			boost::shared_ptr<shapefile> res;

			using boost::filesystem::path;
			using boost::filesystem::exists;

			path pHashShapes;
			path pHashStats;
			shapeFileStats::getHashPaths(pHashShapes, pHashStats);

			path pHashShape = findHash(pHashShapes, hash);
			if (!pHashShape.empty())
				res = boost::shared_ptr<shapefile>(new shapefile(pHashShape.string()));
			//else if (Ryan_Serialization::detect_compressed(_shp->filename))
			//	res = boost::shared_ptr<shapefile>(new shapefile(_shp->filename));
			else
				throw rtmath::debug::xMissingFile(hash.c_str());
			return res;
		}

		void shapefile::writeToHash() const
		{
			using boost::filesystem::path;

			path pHashShapes;
			path pHashStats;
			shapeFileStats::getHashPaths(pHashShapes, pHashStats);

			path pHashShape = storeHash(pHashShapes, _localhash);
			// If a shape matching the hash already exists, there is no need to write an identical file
			if (!Ryan_Serialization::detect_compressed(pHashShape.string()))
				write(pHashShape.string(), true);
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
			std::istringstream ss(s);

			
			boost::iostreams::filtering_istream sin;
			// s can contain either compressed or uncompressed input at this point.
			if (cmeth.size())
				prep_decompression(cmeth, sin);
			sin.push(ss);
			//sin.push(a);
			read(sin);
		}

		void shapefile::readHeader(std::istream &in)
		{
			using namespace std;
			_init();

			// Do header processing using istreams.
			// The previous method used strings, but this didn't work with compressed reads.

			// The header is seven lines long
			for (size_t i=0; i<7; i++)
			{
				string lin;
				std::getline(in,lin);
				size_t posa = 0, posb = 0;
				Eigen::Array3f *v = nullptr;
				switch (i)
				{
				case 0: // Title line
					desc = lin;
					break;
				case 1: // Number of dipoles
					{
						// Seek to first nonspace character
						posa = lin.find_first_not_of(" \t\n", posb);
						// Find first space after this position
						posb = lin.find_first_of(" \t\n", posa);
						size_t len = posb - posa;
						numPoints = rtmath::macros::m_atoi(&(lin.data()[posa]),len);
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
							posa = lin.find_first_not_of(" \t\n,", posb);
							// Find first space after this position
							posb = lin.find_first_of(" \t\n,", posa);
							size_t len = posb - posa;
							(*v)(j) = (float) rtmath::macros::m_atof(&(lin.data()[posa]),len);
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

		void shapefile::read(std::istream &iin)
		{
			// Since istringstream is so slow, I'm dusting off my old atof macros (in 
			// macros.h). These were used when I implemented lbl, and are very fast.
			readHeader(iin);

			using namespace std;
			const size_t numThreads = rtmath::debug::getConcurrentThreadsSupported();

			//Eigen::Vector3f crdsm, crdsi; // point location and diel entries
			set<size_t> mediaIds;
			//size_t posa = 0, posb = 0; //pend+1;
			// Load in the lattice points through iteration and macro.h-based double extraction
			size_t lastmedia = 0; // Used to speed up tree searches in mediaIds
			// TODO: parallelize this function
			for (size_t i=0; i< numPoints; i++)
			{
				string in;
				in.reserve(100);
				// TODO: avoid the line allocation entirely
				std::getline(iin,in);
				size_t posa = 0, posb = 0; //pend+1;
				auto crdsm = latticePts.block<1,3>(i,0);
				auto crdsi = latticePtsRi.block<1,3>(i,0);
				for (size_t j=0; j<7; j++)
				{
					// Seek to first nonspace character
					posa = in.find_first_not_of(" \t\n\0", posb);
					// Find first space after this position
					posb = in.find_first_of(" \t\n\0", posa);
					size_t len = posb - posa;
					float val;
					val = (float) rtmath::macros::m_atof(&(in.data()[posa]),len);
					if (j==0) continue;
					if (j<=3) crdsm(j-1) = val;
					else crdsi(j-4) = val;
				}

				auto checkMedia = [&mediaIds, &lastmedia](size_t id)
				{
					if (id == lastmedia) return;
					if (!mediaIds.count(id))
						mediaIds.insert(id);
					lastmedia = id;
				};

				checkMedia( static_cast<size_t>(crdsi(0)) );
				checkMedia( static_cast<size_t>(crdsi(1)) );
				checkMedia( static_cast<size_t>(crdsi(2)) );
			}

			Dielectrics = mediaIds;
			
			// Figure out third lattice vector in target frame
			a3(0) = a1(1)*a2(2)-a1(2)*a2(1);
			a3(1) = a1(2)*a2(0)-a1(0)*a2(2);
			a3(2) = a1(0)*a2(1)-a1(1)*a2(0);

			// Do a second pass and generate the lattice from the lattice points
			// The scaling factors and basis vectors are already in place.
			xd = x0 * d;

			using namespace boost::accumulators;
			vector<accumulator_set<float, stats<
				tag::min,
				tag::max, 
				tag::mean,
				tag::count> > > 
				 // These only need mean and count, but are here for ease in typing lambdas.
				m_x(numThreads), m_y(numThreads), m_z(numThreads),
				// These need all parameters.
				r_x(numThreads), r_y(numThreads), r_z(numThreads);
			accumulator_set<float, stats<
				tag::min,
				tag::max, 
				tag::mean,
				tag::count> > sr_x, sr_y, sr_z;

			/**
			 * \brief Calculate preliminary stats.
			 * \param index is the accumulator set index in the vector to use.
			 * \param start is the start of the point range to process.
			 * \param end is the end of the point range to process.
			 **/
			auto prelimStats = [&](size_t index, size_t start, size_t end)
			{
				//for (auto it = latticePts.begin(); it != latticePts.end(); ++it)
				for (size_t i=start; i< end; i++)
				{
					auto crdsm = latticePts.block<1,3>(i,0);
					//auto crdsi = latticePtsRi.block<1,3>(i,0);
					// Do componentwise multiplication to do scaling
					Eigen::Array3f crd = crdsm.array() * d.transpose();
					auto crdsc = latticePtsStd.block<1,3>(i,0);
					//Eigen::Vector3f -> next line
					crdsc = crd.matrix() - xd.matrix(); // Normalized coordinates!

					// Need to do stat collection here because the midpoint is usually not set correctly!

					r_x[index](crdsm(0));
					r_y[index](crdsm(1));
					r_z[index](crdsm(2));

					m_x[index](crdsc(0));
					m_y[index](crdsc(1));
					m_z[index](crdsc(2));

					// Save in latticePtsStd
					//latticePtsStd.push_back(move(crdsc));
				}
			};
			
			// Process in threads using sets of 500 points each. Threads are in a pool until all points are processed.
			std::vector<std::pair<size_t, size_t> > cands_init, cands;
			size_t i=0;
			while (i<numPoints)
			{
				size_t maxBound = i + 500;
				if (maxBound > numPoints) maxBound = numPoints;
				cands_init.push_back(std::pair<size_t, size_t>(i, maxBound));
				i = maxBound;
			}
			cands = cands_init;

			std::mutex m_pool;
			auto process_pool_prelim = [&](size_t index)
			{
				try {
					std::pair<size_t, size_t> p;
					for (;;)
					{
						{
							std::lock_guard<std::mutex> lock(m_pool);

							if (!cands.size()) return;
							p = cands.back();
							cands.pop_back();
						}
					
						prelimStats(index, p.first, p.second);
					}
				} catch (std::exception &e)
				{
					std::cerr << e.what() << std::endl;
					return;
				}
			};
			std::vector<std::thread> pool;
			for (size_t i=0; i<numThreads;i++)
			{
				std::thread t(process_pool_prelim,i);
				pool.push_back(std::move(t));
			}
			for (size_t i=0; i<numThreads;i++)
			{
				pool[i].join();
			}



			// Combine the stat entries
			auto findMean = [&](
				vector<accumulator_set<float, stats< tag::min, tag::max, tag::mean, tag::count> > >
				&mSrc, float &mMean)
			{
				for (auto &ac : mSrc)
				mMean += boost::accumulators::mean(ac) * (float) boost::accumulators::count(ac) / (float) numPoints;
			};
			float mm_x = 0, mm_y = 0, mm_z = 0;
			findMean(m_x, mm_x); findMean(m_y, mm_y); findMean(m_z, mm_z);
			float mr_x = 0, mr_y = 0, mr_z = 0;
			findMean(r_x, mr_x); findMean(r_y, mr_y); findMean(r_z, mr_z);
			
			means(0) = mr_x;
			means(1) = mr_y;
			means(2) = mr_z;

			// Pass through stats and get mins and maxs
			auto findMinsMaxs = [&](vector<accumulator_set<float, stats< tag::min, tag::max, tag::mean, tag::count> > >
				&mSrc, size_t index)
			{
				for (auto it = mSrc.cbegin(); it != mSrc.cend(); ++it)
				{
					if (it == mSrc.cbegin())
					{
						mins(index) = boost::accumulators::min(*it);
						maxs(index) = boost::accumulators::max(*it);
					} else {
						if (mins(index) > boost::accumulators::min(*it)) mins(index) = boost::accumulators::min(*it);
						if (maxs(index) < boost::accumulators::max(*it)) mins(index) = boost::accumulators::max(*it);
					}
				}
			};
			findMinsMaxs(r_x, 0); findMinsMaxs(r_y, 1); findMinsMaxs(r_z, 2);

			/// Need to renormalize data points. Mean should be at 0, 0, 0 for plotting!
			auto postStats = [&](size_t start, size_t end)
			{
				//for (auto it = latticePtsStd.begin(); it != latticePtsStd.end(); it++)
				for (size_t i=start; i< end; i++)
				{
					auto pt = latticePts.block<1,3>(i,0);
					auto Npt = latticePtsNorm.block<1,3>(i,0);
					//Eigen::Vector3f pt = *it;
					Npt(0) = pt(0) - mm_x;
					Npt(1) = pt(1) - mm_y;
					Npt(2) = pt(2) - mm_z;
					//latticePtsNorm.push_back(move(pt));
				}
			};
			auto process_pool_post = [&]()
			{
				try {
					std::pair<size_t, size_t> p;
					for (;;)
					{
						{
							std::lock_guard<std::mutex> lock(m_pool);

							if (!cands.size()) return;
							p = cands.back();
							cands.pop_back();
						}
					
						postStats(p.first, p.second);
					}
				} catch (std::exception &e)
				{
					std::cerr << e.what() << std::endl;
					return;
				}
			};
			// Refresh the pool of points to be processed
			cands = cands_init;
			pool.clear();
			for (size_t i=0; i<numThreads;i++)
			{
				std::thread t(process_pool_post);
				pool.push_back(std::move(t));
			}
			for (size_t i=0; i<numThreads;i++)
			{
				pool[i].join();
			}

			
			hash();
		}

		void shapefile::write(std::ostream &out) const
		{
			print(out);
		}

		void shapefile::writeVTK(const std::string &fname) const
		{
			rtmath::ddscat::convexHull hull(latticePtsStd);
			hull.writeVTKraw(fname);
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

			/// \todo Check file extension for a vtk file, and save points and surface
			outfile << filename;
			if (cmeth.size()) outfile << "." << cmeth;
			std::string soutfile = outfile.str();

			ofstream out(soutfile.c_str(), ios_base::out | ios_base::binary);
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

