#include "Stdafx-ddscat.h"
#pragma warning( disable : 4996 ) // -D_SCL_SECURE_NO_WARNINGS
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/spirit/include/karma.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>

#include <Ryan_Serialization/serialization.h>

#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/ddscat/shapestats.h"
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
	bool parse_shapefile_entries(Iterator first, Iterator last, std::vector<long>& v)
	{
		using qi::double_;
		using qi::long_;
		using qi::phrase_parse;
		using qi::_1;
		using ascii::space;
		using phoenix::push_back;

		bool r = phrase_parse(first, last,

			//  Begin grammar
			(
			// *long_[push_back(phoenix::ref(v), _1)]
			*long_
			)
			,
			//  End grammar

			space, v);

		if (first != last) // fail if we did not get a full match
			return false;
		return r;
	}

	/// Used in quickly printing shapefile
	template <typename OutputIterator, typename Container>
	bool print_shapefile_entries(OutputIterator& sink, Container const& v)
	{
		using boost::spirit::karma::long_;
		using boost::spirit::karma::repeat;
		using boost::spirit::karma::generate;
		//using boost::spirit::karma::generate_delimited;
		using boost::spirit::ascii::space;

		bool r = generate(
			sink,                           // destination: output iterator
			*(
			//repeat(7)()
			'\t' << long_ << '\t' << // point id
			long_ << '\t' << long_ << '\t' << long_ << '\t' << // point coordinates
			long_ << '\t' << long_ << '\t' << long_ << '\n' // dielectric
			),
			//space,                          // the delimiter-generator
			v                               // the data to output 
			);
		return r;
	}
}



namespace rtmath
{
	namespace ddscat
	{
		namespace shapefile
		{

			/// \todo Remove dependency on boost accumulators for faster compilation
			void shapefile::readContents(const char *iin, size_t headerEnd)
			{
				// Since istringstream is so slow, I'm dusting off my old atof macros (in 
				// macros.h). These were used when I implemented lbl, and are very fast.
				//size_t headerEnd = 0;
				//readHeader(iin, headerEnd);

				// Figure out third lattice vector in target frame
				a3(0) = a1(1)*a2(2) - a1(2)*a2(1);
				a3(1) = a1(2)*a2(0) - a1(0)*a2(2);
				a3(2) = a1(0)*a2(1) - a1(1)*a2(0);
				xd = x0 * d;

				using namespace std;
				// Cannot easily multithread due to file format restrictions.
				//const size_t numThreads = rtmath::debug::getConcurrentThreadsSupported();

				//Eigen::Vector3f crdsm, crdsi; // point location and diel entries
				const char* pa = &iin[headerEnd];
				const char* pb = strchr(pa + 1, '\0');

				std::vector<long> parser_vals; //(numPoints*8);
				parser_vals.reserve(numPoints*8);
				parse_shapefile_entries(pa,pb, parser_vals);


				using namespace boost::accumulators;
				accumulator_set<float, boost::accumulators::stats<tag::mean, tag::min, tag::max> > m_x, m_y, m_z;
				size_t lastmedia = 0; // Used to speed up tree searches in mediaIds
				set<size_t> mediaIds;

				for (size_t i = 0; i < numPoints; ++i)
				{
					// First field truly is a dummy variable. No correclation with point ordering at all.
					//size_t pIndex = parser_vals[index].at(7 * i) - 1;
					size_t pIndex = 7*i;
					auto crdsm = latticePts.block<1, 3>(i, 0);
					auto crdsi = latticePtsRi.block<1, 3>(i, 0);
					latticeIndex(i) = (int) parser_vals.at(pIndex);
					for (size_t j = 1; j < 7; j++) // TODO: rewrite using eigen?
					{
						float val = (float)parser_vals.at(pIndex + j);
						if (j <= 3) crdsm(j - 1) = val;
						else crdsi(j - 4) = val;
					}

					auto checkMedia = [&mediaIds, &lastmedia](size_t id)
					{
						if (id == lastmedia) return;
						if (!mediaIds.count(id))
							mediaIds.insert(id);
						lastmedia = id;
					};

					checkMedia(static_cast<size_t>(crdsi(0)));
					checkMedia(static_cast<size_t>(crdsi(1)));
					checkMedia(static_cast<size_t>(crdsi(2)));

					Eigen::Array3f crd = crdsm.array() * d.transpose();
					auto crdsc = latticePtsStd.block<1, 3>(i, 0);
					//Eigen::Vector3f -> next line
					crdsc = crd.matrix() - xd.matrix(); // Normalized coordinates!

					// Need to do stat collection here because the midpoint is usually not set correctly!
					m_x(crdsm(0));
					m_y(crdsm(1));
					m_z(crdsm(2));
					/*
					r_x(crdsc(0));
					r_y(crdsc(1));
					r_z(crdsc(2));
					*/
				}
				for (auto id : mediaIds)
					Dielectrics.emplace(id);


				mins(0) = boost::accumulators::min(m_x);
				mins(1) = boost::accumulators::min(m_y);
				mins(2) = boost::accumulators::min(m_z);

				maxs(0) = boost::accumulators::max(m_x);
				maxs(1) = boost::accumulators::max(m_y);
				maxs(2) = boost::accumulators::max(m_z);

				means(0) = boost::accumulators::mean(m_x);
				means(1) = boost::accumulators::mean(m_y);
				means(2) = boost::accumulators::mean(m_z);


				/// Need to renormalize data points. Mean should be at 0, 0, 0 for plotting!
				for (size_t i = 0; i < numPoints; i++)
				{
					auto pt = latticePts.block<1, 3>(i, 0);
					auto Npt = latticePtsNorm.block<1, 3>(i, 0);
					Npt = pt.array().transpose() - means;
				}
				

				hash();
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
				//size_t i = 1;

				std::vector<long> oi(numPoints * 7);

				for (size_t j = 0; j < numPoints; j++)
				{
					long point = latticeIndex(j);
					auto it = latticePts.block<1, 3>(j, 0);
					auto ot = latticePtsRi.block<1, 3>(j, 0);
					oi[j * 7 + 0] = point;
					oi[j * 7 + 1] = (long)(it)(0);
					oi[j * 7 + 2] = (long)(it)(1);
					oi[j * 7 + 3] = (long)(it)(2);
					oi[j * 7 + 4] = (long)(ot)(0);
					oi[j * 7 + 5] = (long)(ot)(1);
					oi[j * 7 + 6] = (long)(ot)(2);

					//out << "\t" << i << "\t";
					//out << (it)(0) << "\t" << (it)(1) << "\t" << (it)(2) << "\t";
					//out << (ot)(0) << "\t" << (ot)(1) << "\t" << (ot)(2);
					//out << endl;
				}

				std::string generated;
				std::back_insert_iterator<std::string> sink(generated);
				if (!print_shapefile_entries(sink, oi))
				{
					// generating failed
					cerr << "Generating failed\n";
					throw;
				}
				out << generated;
			}

			void shapefile::recalcStats()
			{
				using namespace std;
				const size_t numThreads = rtmath::debug::getConcurrentThreadsSupported();

				std::vector<std::thread> pool;
				std::mutex m_pool, m_media;

				xd = x0 * d;

				// Process in threads using sets of 500 points each. Threads are in a pool until all points are processed.
				std::vector<std::pair<size_t, size_t> > cands_init, cands;
				size_t i = 0;
				while (i<numPoints)
				{
					size_t maxBound = i + 500;
					if (maxBound > numPoints) maxBound = numPoints;
					cands_init.push_back(std::pair<size_t, size_t>(i, maxBound));
					i = maxBound;
				}
				cands = cands_init;


				using namespace boost::accumulators;
				vector<accumulator_set<float, boost::accumulators::stats<
					tag::min,
					tag::max,
					tag::mean,
					tag::count> > >
					// These only need mean and count, but are here for ease in typing lambdas.
					m_x(numThreads), m_y(numThreads), m_z(numThreads),
					// These need all parameters.
					r_x(numThreads), r_y(numThreads), r_z(numThreads);
				accumulator_set<float, boost::accumulators::stats<
					tag::min,
					tag::max,
					tag::mean,
					tag::count> > sr_x, sr_y, sr_z;



				auto process_pool_raws_import = [&](size_t index, size_t start, size_t end)
				{
					size_t lastmedia = 0; // Used to speed up tree searches in mediaIds
					set<size_t> mediaIds;

					for (size_t i = start; i < end; ++i)
					{
						auto crdsm = latticePts.block<1, 3>(i, 0);
						auto crdsi = latticePtsRi.block<1, 3>(i, 0);

						Eigen::Array3f crd = crdsm.array() * d.transpose();
						auto crdsc = latticePtsStd.block<1, 3>(i, 0);
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
						std::pair<size_t, size_t> p;
						for (;;)
						{
							{
								std::lock_guard<std::mutex> lock(m_pool);

								if (!cands.size()) return;
								p = cands.back();
								cands.pop_back();
							}

							process_pool_raws_import(i, p.first, p.second);
						}
					}
					catch (std::exception &e)
					{
						std::cerr << e.what() << std::endl;
						return;
					}
				};
				for (size_t i = 0; i < numThreads; i++)
				{
					std::thread t(process_pool_raws, i);
					pool.push_back(std::move(t));
				}
				for (size_t i = 0; i < numThreads; i++)
				{
					pool[i].join();
				}
				pool.clear();

				cands = cands_init;

				// Combine the stat entries
				auto findMean = [&](
					vector<accumulator_set<float, boost::accumulators::stats< tag::min, tag::max, tag::mean, tag::count> > >
					&mSrc, float &mMean)
				{
					for (auto &ac : mSrc)
						mMean += boost::accumulators::mean(ac) * (float)boost::accumulators::count(ac) / (float)numPoints;
				};
				float mm_x = 0, mm_y = 0, mm_z = 0;
				findMean(m_x, mm_x); findMean(m_y, mm_y); findMean(m_z, mm_z);
				float mr_x = 0, mr_y = 0, mr_z = 0;
				findMean(r_x, mr_x); findMean(r_y, mr_y); findMean(r_z, mr_z);

				means(0) = mr_x;
				means(1) = mr_y;
				means(2) = mr_z;

				// Pass through stats and get mins and maxs
				auto findMinsMaxs = [&](vector<accumulator_set<float, boost::accumulators::stats< tag::min, tag::max, tag::mean, tag::count> > >
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
						}
						else {
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
					for (size_t i = start; i < end; i++)
					{
						auto pt = latticePts.block<1, 3>(i, 0);
						auto Npt = latticePtsNorm.block<1, 3>(i, 0);
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
					}
					catch (std::exception &e)
					{
						std::cerr << e.what() << std::endl;
						return;
					}
				};
				// Refresh the pool of points to be processed
				cands = cands_init;
				pool.clear();
				for (size_t i = 0; i < numThreads; i++)
				{
					std::thread t(process_pool_post);
					pool.push_back(std::move(t));
				}
				for (size_t i = 0; i < numThreads; i++)
				{
					pool[i].join();
				}


				hash();
			}

			boost::shared_ptr<shapefile> shapefile::loadHash(
				const std::string &hash)
			{
				boost::shared_ptr<shapefile> res;

				using boost::filesystem::path;
				using boost::filesystem::exists;

				path pHashShapes;
				path pHashStats;
				rtmath::ddscat::stats::shapeFileStats::getHashPaths(pHashShapes, pHashStats);

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
				rtmath::ddscat::stats::shapeFileStats::getHashPaths(pHashShapes, pHashStats);

				path pHashShape = storeHash(pHashShapes, _localhash);
				if (pHashShape.string() == "") return; // Silently fail (no hash dir)
				// If a shape matching the hash already exists, there is no need to write an identical file
				if (!Ryan_Serialization::detect_compressed(pHashShape.string()))
					write(pHashShape.string(), true);
			}

		}
	}
}
