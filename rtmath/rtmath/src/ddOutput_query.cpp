#include "Stdafx-ddscat.h"
#pragma warning( disable : 4996 ) // -D_SCL_SECURE_NO_WARNINGS
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <tuple>
#include <boost/math/constants/constants.hpp>
#include <boost/lexical_cast.hpp>

#include <Ryan_Debug/debug.h>
#include "../rtmath/macros.h"
#include "../rtmath/hash.h"
#include "../rtmath/ddscat/ddOutput.h"
#include "../rtmath/splitSet.h"
#include "../rtmath/registry.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"

namespace rtmath {

	namespace ddscat {

			bool ddOutput_db_registry::ddOutput_db_comp::operator()(const std::shared_ptr<ddOutput>& lhs, const std::shared_ptr<ddOutput>& rhs) const
			{
				return *lhs < *rhs;
			}

			bool ddOutput_db_registry::ddOutput_db_comp::operator()(const boost::shared_ptr<ddOutput>& lhs, const boost::shared_ptr<ddOutput>& rhs) const
			{
				return *lhs < *rhs;
			}


			bool ddOutput::operator!=(const ddOutput &rhs) const { return !operator==(rhs); }
			bool ddOutput::operator==(const ddOutput &rhs) const { 
#define check(x) if (x != rhs.x) return false;
				check(shapeHash);
				check(freq); check(temp); check(aeff); check(numOriData);
				return true;
			}
			bool ddOutput::operator<(const ddOutput &rhs) const { 
#define check(x) if (x != rhs.x) return x < rhs.x;
				check(shapeHash);
				check(freq); check(temp); check(aeff); check(numOriData);
				return false;
#undef check
			}

			ddOutput_db_registry::ddOutput_db_registry() : name(nullptr), fQuery(nullptr) {}
			ddOutput_db_registry::~ddOutput_db_registry() {}

			ddOutput_db_registry::ddOutput_index::ddOutput_index() {}
			ddOutput_db_registry::ddOutput_index::~ddOutput_index() {}
			std::shared_ptr<ddOutput_db_registry::ddOutput_index>
				ddOutput_db_registry::ddOutput_index::generate()
			{
				std::shared_ptr<ddOutput_db_registry::ddOutput_index> res
					(new ddOutput_index());
				return res;
			}

#define searchAndVec1(fname, varname, t) \
	ddOutput_db_registry::ddOutput_index& ddOutput_db_registry::ddOutput_index::fname(const t& s) \
						{ \
				varname.insert(boost::lexical_cast<std::string>(s)); \
				return *this; \
						} \
			ddOutput_db_registry::ddOutput_index& ddOutput_db_registry::ddOutput_index::fname(const std::vector<t>& s) \
						{ \
				for (const auto &i : s) \
					varname.insert(boost::lexical_cast<std::string>(i)); \
				return *this; \
						}
#define searchAndVec2(fname, varname, t) \
	ddOutput_db_registry::ddOutput_index& ddOutput_db_registry::ddOutput_index::fname(const t s) \
						{ \
				varname.insert(boost::lexical_cast<std::string>(s)); \
				return *this; \
						}

			ddOutput_db_registry::ddOutput_index& ddOutput_db_registry::ddOutput_index::dipoleRange(
				size_t lower, size_t upper)
			{
				dipoleRanges.push_back(std::pair<size_t, size_t>(lower, upper)); return *this;
			}
			ddOutput_db_registry::ddOutput_index& ddOutput_db_registry::ddOutput_index::dipoleRange(
				const std::vector<std::pair<size_t, size_t> > &vec)
			{
				for (const auto & v : vec)
					dipoleRanges.push_back(v);
				return *this;
			}
			ddOutput_db_registry::ddOutput_index& ddOutput_db_registry::ddOutput_index::betaRange(
				size_t lower, size_t upper)
			{
				betaRange.push_back(std::pair<size_t, size_t>(lower, upper)); return *this;
			}
			ddOutput_db_registry::ddOutput_index& ddOutput_db_registry::ddOutput_index::betaRange(
				const std::vector<std::pair<size_t, size_t> > &vec)
			{
				for (const auto & v : vec)
					betaRange.push_back(v);
				return *this;
			}
			ddOutput_db_registry::ddOutput_index& ddOutput_db_registry::ddOutput_index::thetaRange(
				size_t lower, size_t upper)
			{
				thetaRange.push_back(std::pair<size_t, size_t>(lower, upper)); return *this;
			}
			ddOutput_db_registry::ddOutput_index& ddOutput_db_registry::ddOutput_index::thetaRange(
				const std::vector<std::pair<size_t, size_t> > &vec)
			{
				for (const auto & v : vec)
					thetaRange.push_back(v);
				return *this;
			}
			ddOutput_db_registry::ddOutput_index& ddOutput_db_registry::ddOutput_index::phiRange(
				size_t lower, size_t upper)
			{
				phiRange.push_back(std::pair<size_t, size_t>(lower, upper)); return *this;
			}
			ddOutput_db_registry::ddOutput_index& ddOutput_db_registry::ddOutput_index::phiRange(
				const std::vector<std::pair<size_t, size_t> > &vec)
			{
				for (const auto & v : vec)
					phiRange.push_back(v);
				return *this;
			}
			ddOutput_db_registry::ddOutput_index& ddOutput_db_registry::ddOutput_index::aeffRange(
				float lower, float upper)
			{
				aeffRanges.push_back(std::pair<float, float>(lower, upper)); return *this;
			}
			ddOutput_db_registry::ddOutput_index& ddOutput_db_registry::ddOutput_index::aeffRange(
				const std::vector<std::pair<float, float> > &vec)
			{
				for (const auto & v : vec)
					aeffRanges.push_back(v);
				return *this;
			}
			ddOutput_db_registry::ddOutput_index& ddOutput_db_registry::ddOutput_index::tempRange(
				float lower, float upper)
			{
				tempRanges.push_back(std::pair<float, float>(lower, upper)); return *this;
			}
			ddOutput_db_registry::ddOutput_index& ddOutput_db_registry::ddOutput_index::tempRange(
				const std::vector<std::pair<float, float> > &vec)
			{
				for (const auto & v : vec)
					tempRanges.push_back(v);
				return *this;
			}

			//searchAndVec1(tag, tags, std::string);
			searchAndVec1(hashLower, hashLowers, std::string);
			searchAndVec1(hashUpper, hashUppers, std::string);
			searchAndVec1(flakeType, flakeTypes, std::string);
			searchAndVec1(runId, runids, std::string);
			searchAndVec1(polarization, polarization, std::string);
			//searchAndVec1(hash, hashLowers, HASH_t);
			searchAndVec2(hashLower, hashLowers, uint64_t);
			searchAndVec2(hashUpper, hashUppers, uint64_t);

			ddOutput_db_registry::ddOutput_index& ddOutput_db_registry::ddOutput_index::standardD(const float d, const float tolpercent)
			{
				standardDs.insert(std::pair<float, float>(d, tolpercent));
				return *this;
			}
			ddOutput_db_registry::ddOutput_index& ddOutput_db_registry::ddOutput_index::freqRange(const float d, const float tolpercent)
			{
				freqRanges.insert(std::pair<float, float>(d, tolpercent));
				return *this;
			}

#undef searchAndVec1
#undef searchAndVec2

			std::pair<ddOutput_db_registry::ddOutput_index::collection, std::shared_ptr<rtmath::registry::DBhandler> >
				ddOutput_db_registry::ddOutput_index::doQuery(std::shared_ptr<rtmath::registry::DBhandler> p, std::shared_ptr<registry::DB_options> o) const
			{
				collection c(new std::set<boost::shared_ptr<ddOutput>, ddOutput_db_comp >());
				std::shared_ptr<rtmath::registry::DBhandler> fp;

				auto hooks = ::rtmath::registry::usesDLLregistry<ddOutput_query_registry, ddOutput_db_registry >::getHooks();
				for (const auto &h : *(hooks.get()))
				{
					if (!h.fQuery) continue;
					if (!h.fMatches) continue;
					if (!h.fMatches(p, o)) continue;
					fp = h.fQuery(*this, c, p, o);

					return std::pair < ddOutput_db_registry::ddOutput_index::collection,
						std::shared_ptr<rtmath::registry::DBhandler> >(c, fp);
				}

				return std::pair<ddOutput_db_registry::ddOutput_index::collection,
					std::shared_ptr<rtmath::registry::DBhandler> >
					(c, nullptr);
			}


			std::pair<ddOutput_db_registry::ddOutput_index::collection,
				std::shared_ptr<rtmath::registry::DBhandler> >
				ddOutput_db_registry::ddOutput_index::doQuery(
				collection srcs, bool doUnion, bool doDb,
				std::shared_ptr<rtmath::registry::DBhandler> p,
				std::shared_ptr<registry::DB_options> o) const
			{
				collection toMergeC(new std::set<boost::shared_ptr<ddOutput>, ddOutput_db_comp >());
				//collection toMergeS(new std::set<boost::shared_ptr<ddOutput>, ddOutput_db_comp >());
				collection res(new std::set<boost::shared_ptr<ddOutput>, ddOutput_db_comp >());
				std::shared_ptr<rtmath::registry::DBhandler> fp;

				std::map<rtmath::HASH_t, boost::shared_ptr<ddOutput> > db_hashes; // database results, as a map

				auto hooks = ::rtmath::registry::usesDLLregistry<ddOutput_query_registry, ddOutput_db_registry >::getHooks();
				if (doDb)
				{
					for (const auto &h : *(hooks.get()))
					{
						if (!h.fQuery) continue;
						if (!h.fMatches) continue;
						if (!h.fMatches(p, o)) continue;
						fp = h.fQuery(*this, toMergeC, p, o);

						//return std::pair < ddOutput_db_registry::ddOutput_index::collection,
						//	std::shared_ptr<rtmath::registry::DBhandler> >(c, fp);
					}

					for (const auto &r : *(toMergeC))
						db_hashes[r->hash()] = r;
				}



				// Also perform the same filtering on srcs, putting results in res
				for (auto &s : *(srcs))
				{
					auto works = [&]() -> bool
					{
						// Tag filtering - OR filtering
						if (tags.size())
						{
							bool matches = false;
							for (const auto &t : tags)
							{
								if (s->tags.count(t.first))
								{
									if (s->tags[t.first] == t.second)
										matches = true;
									break;
								}
							}
							if (!matches) return false;
						}

						// hash filtering
						if (hashLowers.size() && !hashLowers.count(s->hash().string()))
							return false;
						if (hashUppers.size() && !hashUppers.count(boost::lexical_cast<std::string>(s->hash().upper)))
							return false;

						// flake types
						if (flakeTypes.size())
						{
							if (s->tags.count("flake_classification"))
							{
								if (!flakeTypes.count(s->tags.at("flake_classification")))
									return false;
							}
						}

						// refHashLowers
						if (s->tags.count("flake_reference"))
						{
							if (refHashLowers.size() &&
								std::find(refHashLowers.begin(), refHashLowers.end(),
								boost::lexical_cast<std::string>(s->hash().lower)) == refHashLowers.end())
								return false;
						}

						// Dipole spacings
						if (standardDs.size())
						{
							bool matches = false;
							for (const auto &r : standardDs)
							{
								if (r.first * (1. - r.second) <= s->standardD && s->standardD <= r.first * (1. + r.second))
								{
									matches = true;
									break;
								}
							}
							if (!matches) return false;
						}

						// Number of dipoles
						if (dipoleRanges.size())
						{
							bool matches = false;
							for (const auto &r : dipoleRanges)
							{
								if (r.first <= s->numPoints && s->numPoints <= r.second)
								{
									matches = true;
									break;
								}
							}
							if (!matches) return false;
						}
						return true;
					};
					if (!works()) continue;

					res->insert(s); // Passed filtering
					// Merge the results of the provided object with any query results
					if (db_hashes.count(s->hash()))
					{
						auto d = db_hashes.at(s->hash());

						if (!s->standardD && d->standardD) s->standardD = d->standardD;
						if (!s->numPoints && d->numPoints) s->numPoints = d->numPoints;
						if (!s->desc.size() && d->desc.size()) s->desc = d->desc;
						for (const auto &tag : d->tags)
							if (!s->tags.count(tag.first)) s->tags[tag.first] = tag.second;

						db_hashes.erase(s->hash()); // Remove from consideration (already matched)
					}
				}

				// If doUnion, select the remainder of doHashes and insert
				if (doUnion)
					for (const auto &d : db_hashes)
						res->insert(d.second);


				return std::pair<ddOutput_db_registry::ddOutput_index::collection,
					std::shared_ptr<rtmath::registry::DBhandler> >
					(res, fp);
			}
		}
	
}

