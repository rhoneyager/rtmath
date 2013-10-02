#include "Stdafx-ddscat.h"
#pragma warning( disable : 4996 ) // -D_SCL_SECURE_NO_WARNINGS
#include <iostream>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <thread>
#include <mutex>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <boost/filesystem.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>

#include <Ryan_Serialization/serialization.h>
#include "../rtmath/macros.h"
#include "../rtmath/hash.h"
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/ddscat/shapestats.h"
#include "../rtmath/ddscat/hulls.h" // Hulls and VTK functions
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"

/// Internal namespace for the reader parsers
namespace {
	namespace qi = boost::spirit::qi;
	namespace ascii = boost::spirit::ascii;
	namespace phoenix = boost::phoenix;

	/** \brief Parses space-separated shapefile entries.
	**/
	template <typename Iterator>
	bool parse_shapefile_entries(Iterator first, Iterator last, std::vector<unsigned long>& v)
	{
		using qi::double_;
		using qi::ulong_;
		using qi::phrase_parse;
		using qi::_1;
		using ascii::space;
		using phoenix::push_back;

		bool r = phrase_parse(first, last,

			//  Begin grammar
			(
			// *ulong_[push_back(phoenix::ref(v), _1)]
			*ulong_
			)
			,
			//  End grammar

			space, v);

		if (first != last) // fail if we did not get a full match
			return false;
		return r;
	}

	/*
	struct parse_stats
	{
		parse_stats() : mins(3,0), maxs(3,0), means(3,0) {}
		std::vector<double> mins, maxs, means;
	};

	template <typename Iterator>
	bool parse_shapefile_entries_new(Iterator first, Iterator last, std::vector<unsigned long>& v, parse_stats &s, size_t n)
	{
		using qi::double_;
		using qi::ulong_;
		using qi::phrase_parse;
		using qi::_1;
		using qi::_2;
		using ascii::space;
		using phoenix::ref;
		using phoenix::push_back;

		bool r = phrase_parse(first, last,

			//  Begin grammar
			(
			*(
			// Cell ID
			double_[push_back(phoenix::ref(v), _1)] >> 
			// X
			double_ >> 
			// Y
			double_ >> 
			// Z
			double_ >> 
			// iX
			ulong_ >> 
			// iY
			ulong_ >> 
			// iZ
			ulong_
			)
			// *ulong_[push_back(phoenix::ref(v), _1)]
			//*ulong_
			)
			,
			//  End grammar

			space, v);

		if (first != last) // fail if we did not get a full match
			return false;
		return r;
	}

	*/
}

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
			std::ostringstream so;
			boost::iostreams::copy(in, so);
			std::string s;
			s = so.str();
			this->_localhash = HASH(s.c_str(),(int) s.size());

			std::istringstream ss_unc(s);
			read(s);
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
			// sin can contain either compressed or uncompressed input at this point.
			if (cmeth.size())
				prep_decompression(cmeth, sin);
			sin.push(ss);
			
			string suncompressed;
			suncompressed.reserve(1024*1024*10);
			std::ostringstream so;
			boost::iostreams::copy(sin, so);
			suncompressed = so.str();
			this->_localhash = HASH(suncompressed.c_str(),(int) suncompressed.size());

			istringstream ss_unc(suncompressed);
			readContents(suncompressed.c_str());
		}

		void shapefile::readHeader(const char* in, size_t &headerEnd)
		{
			using namespace std;
			_init();

			// Do header processing using istreams.
			// The previous method used strings, but this didn't work with compressed reads.
			//size_t &pend = headerEnd;
			const char* pend = in;
			const char* pstart = in;

			// The header is seven lines long
			for (size_t i=0; i<7; i++)
			{
				pstart = pend;
				pend = strchr(pend, '\n');
				pend++; // Get rid of the newline
				//pend = in.find_first_of("\n", pend+1);
				string lin(pstart, pend-pstart-1);
				//std::getline(in,lin);
				
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

			headerEnd = (pend - in) / sizeof(char);
			latticePts.resize(numPoints,3);
			latticePtsRi.resize(numPoints,3);
			latticePtsStd.resize(numPoints,3);
			latticePtsNorm.resize(numPoints,3);
		}

		void shapefile::readContents(const char *iin)
		{
			// Since istringstream is so slow, I'm dusting off my old atof macros (in 
			// macros.h). These were used when I implemented lbl, and are very fast.
			size_t headerEnd = 0;
			readHeader(iin, headerEnd);

			// Figure out third lattice vector in target frame
			a3(0) = a1(1)*a2(2)-a1(2)*a2(1);
			a3(1) = a1(2)*a2(0)-a1(0)*a2(2);
			a3(2) = a1(0)*a2(1)-a1(1)*a2(0);
			xd = x0 * d;

			using namespace std;
			const size_t numThreads = rtmath::debug::getConcurrentThreadsSupported();

			//Eigen::Vector3f crdsm, crdsi; // point location and diel entries
			const char* pa = &iin[headerEnd];
			const char* pb = strchr(pa+1, '\0');

			//std::vector<std::vector<unsigned long>> parser_vals(numThreads);
			//for (auto &pv : parser_vals)
			//	pv.reserve((numPoints * 7));
			//parse_shapefile_entries(pa,pb, parser_vals);

			// Threading the parser to read in and process the points
			std::vector<std::thread> pool;
			std::mutex m_pool, m_media;
			// Create the pool of point ranges to read (partly based on pa and pb range)
			std::vector<std::pair<const char*, const char*> > point_ranges, point_ranges_b;
			point_ranges.reserve(numThreads * 21);
			std::vector<std::vector<unsigned long> > parser_vals(numThreads);
			for (auto &p : parser_vals)
				p.reserve((numPoints * 8) / (numThreads*19));
			const char* pe = pa;
			while (pe < pb)
			{
				const char* ps = pe;
				pe += (pb - pa) / (numThreads * 20);
				if (pe >= pb) pe = pb;
				else {
					pe = strchr(pe, '\n');
				}
				point_ranges.push_back(std::move(std::pair<const char*, const char*>(ps,pe)));
			}
			point_ranges_b = point_ranges;

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


			auto process_pool_raws_import = [&](size_t index, const char* start, const char* end)
			{
				parse_shapefile_entries(start,end, parser_vals[index]);
				size_t lastmedia = 0; // Used to speed up tree searches in mediaIds
				set<size_t> mediaIds;

				for (size_t i=0; i < parser_vals[index].size() / 7; ++i)
				{
					size_t pIndex = parser_vals[index].at(7*i) - 1;
					auto crdsm = latticePts.block<1,3>(pIndex,0);
					auto crdsi = latticePtsRi.block<1,3>(pIndex,0);
					for (size_t j=1; j<7; j++)
					{
						float val = (float) parser_vals[index].at( (7*i) + j );
						//val = (float) rtmath::macros::m_atof(&(in.data()[posa]),len);
						//if (j==0) continue;
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

					Eigen::Array3f crd = crdsm.array() * d.transpose();
					auto crdsc = latticePtsStd.block<1,3>(pIndex,0);
					//Eigen::Vector3f -> next line
					crdsc = crd.matrix() - xd.matrix(); // Normalized coordinates!

					// Need to do stat collection here because the midpoint is usually not set correctly!

					r_x[index](crdsm(0));
					r_y[index](crdsm(1));
					r_z[index](crdsm(2));

					m_x[index](crdsc(0));
					m_y[index](crdsc(1));
					m_z[index](crdsc(2));
				}
				std::lock_guard<std::mutex> lock(m_media);
				for (auto id : mediaIds)
					Dielectrics.emplace(id);
			};

			auto process_pool_raws = [&](size_t i)
			{
				try {
					std::pair<const char*, const char*> p;
					for (;;)
					{
						{
							std::lock_guard<std::mutex> lock(m_pool);

							if (!point_ranges.size()) return;
							p = point_ranges.back();
							point_ranges.pop_back();
						}
					
						process_pool_raws_import(i, p.first, p.second);
					}
				} catch (std::exception &e)
				{
					std::cerr << e.what() << std::endl;
					return;
				}
			};
			for (size_t i=0; i<numThreads;i++)
			{
				std::thread t(process_pool_raws,i);
				pool.push_back(std::move(t));
			}
			for (size_t i=0; i<numThreads;i++)
			{
				pool[i].join();
			}
			pool.clear();
			point_ranges = point_ranges_b;



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
					float mmin = boost::accumulators::min(*it);
					float mmax = boost::accumulators::max(*it);
					if (it == mSrc.cbegin())
					{
						mins(index) = mmin;
						maxs(index) = mmax;
					} else {
						if (mins(index) > mmin) mins(index) = mmin;
						if (maxs(index) < mmax) maxs(index) = mmax;
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

		void shapefile::writeBOV(const std::string &prefix) const
		{
			using namespace boost::filesystem;
			using std::string;
			using std::ofstream;
			string sDataFile = string(prefix).append(".dat");
			string sCtrlFile = string(prefix).append(".bov");
			path pDatafile = path(sDataFile).filename();

			int maxX = static_cast<int>(maxs(0)), maxY = static_cast<int>(maxs(1)), maxZ = static_cast<int>(maxs(2));
			int minX = static_cast<int>(mins(0)), minY = static_cast<int>(mins(1)), minZ = static_cast<int>(mins(2));

			// First, write the control file
			ofstream oct(sCtrlFile.c_str(), std::ios::binary | std::ios::out);
			oct << "TIME: 0\n"
				"DATA_FILE: " << pDatafile.string() << "\n"
				"# The data file size corresponds to the raw flake dimensions\n"
				"DATA_SIZE: "
				<< maxX + 1 - minX << " "
				<< maxY + 1 - minY << " "
				<< maxZ + 1 - minZ << "\n"
				"# Allowable values for DATA_FORMAT are: BYTE,SHORT,INT,FLOAT,DOUBLE\n"
				"DATA_FORMAT: SHORT\n"
				"VARIABLE: Composition\n"
				"# Endian representation of the computer that created the data.\n"
				"# Intel is LITTLE, many other processors are BIG.\n"
				"DATA_ENDIAN: LITTLE\n"
				"# Centering refers to how the data is distributed in a cell. If you\n"
				"# give \"zonal\" then it’s 1 data value per zone. Otherwise the data\n"
				"# will be centered at the nodes.\n"
				"CENTERING: zonal\n"
				"# BRICK_ORIGIN lets you specify a new coordinate system origin for\n"
				"# the mesh that will be created to suit your data.\n"
				"BRICK_ORIGIN: "
				<< static_cast<int>(x0(0)) << " "
				<< static_cast<int>(x0(1)) << " "
				<< static_cast<int>(x0(2)) << "\n"
				"# BRICK_SIZE lets you specify the size of the brick.\n"
				"BRICK_SIZE: 10. 10. 10.\n"
				"# DATA_COMPONENTS: is optional and tells the BOV reader how many\n"
				"# components your data has. 1=scalar, 2=complex number, 3=vector,\n"
				"# 4 and beyond indicate an array variable. You can use \"COMPLEX\"\n"
				"# instead of \"2\" for complex numbers. When your data consists of\n"
				"# multiple components, all components for a cell or node are written\n"
				"# sequentially to the file before going to the next cell or node.\n"
				"DATA_COMPONENTS: 1\n";
			// Then, write the data file
			//ofstream out(sDataFile.c_str(), std::ios::binary | std::ios::out);
			FILE * pOut;
			pOut = fopen(sDataFile.c_str(), "wb");
			// Allocate an array of the correct size
			const size_t sx = static_cast<size_t>(maxX + 1 - minX);
			const size_t sy = static_cast<size_t>(maxY + 1 - minY);
			const size_t sz = static_cast<size_t>(maxZ + 1 - minZ);
			const size_t size = sx * sy * sz;
			std::unique_ptr<short[]> array(new short[size*1]);
			std::fill_n(array.get(), size*1, 0);
			for (size_t i=0; i < numPoints; ++i)
			{
				auto crdsm = latticePts.block<1,3>(i,0);
				float x = crdsm(0), y = crdsm(1), z = crdsm(2);
				auto crdsi = latticePtsRi.block<1,3>(i,0);
				short diel = static_cast<short>(crdsi(0));
				
				size_t start = static_cast<size_t>(x - minX) * sy * sz;
				start += static_cast<size_t>(y - minY) * sz;
				start += static_cast<size_t>(z - minZ);
				start *= 1;

				array[start+0] = diel;
				//array[start+1] = static_cast<short>(crdsi(1));
				//array[start+2] = static_cast<short>(crdsi(2));
			}
			fwrite((void*)array.get(), sizeof(short), size, pOut);
			fclose(pOut);
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

		/// \todo Revamp to provide much faster writes.
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

