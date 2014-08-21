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
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/splitSet.h"
#include "../rtmath/registry.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"

namespace rtmath {
	namespace registry {
		template struct IO_class_registry_writer
			<::rtmath::ddscat::shapefile::shapefile>;

		template struct IO_class_registry_reader
			<::rtmath::ddscat::shapefile::shapefile>;

		template class usesDLLregistry<
			::rtmath::ddscat::shapefile::shapefile_IO_output_registry,
			IO_class_registry_writer<::rtmath::ddscat::shapefile::shapefile> >;

		template class usesDLLregistry<
			::rtmath::ddscat::shapefile::shapefile_IO_input_registry,
			IO_class_registry_reader<::rtmath::ddscat::shapefile::shapefile> >;
		
	}

	namespace ddscat {
		namespace shapefile {

			bool shapefile_db_registry::shapefile_db_comp::operator()(const std::shared_ptr<shapefile>& lhs, const std::shared_ptr<shapefile>& rhs) const
			{
				return *lhs < *rhs;
			}

			bool shapefile_db_registry::shapefile_db_comp::operator()(const boost::shared_ptr<shapefile>& lhs, const boost::shared_ptr<shapefile>& rhs) const
			{
				return *lhs < *rhs;
			}


			bool shapefile::operator!=(const shapefile &rhs) const { return !operator==(rhs); }
			bool shapefile::operator==(const shapefile &rhs) const { return hash().lower == rhs.hash().lower; }
			bool shapefile::operator<(const shapefile &rhs) const { return numPoints < rhs.numPoints; }

			shapefile_db_registry::shapefile_db_registry() : name(nullptr), fQuery(nullptr) {}
			shapefile_db_registry::~shapefile_db_registry() {}

			shapefile_db_registry::shapefile_index::shapefile_index() {}
			shapefile_db_registry::shapefile_index::~shapefile_index() {}
			std::shared_ptr<shapefile_db_registry::shapefile_index> 
				shapefile_db_registry::shapefile_index::generate()
			{
				std::shared_ptr<shapefile_db_registry::shapefile_index> res
					(new shapefile_index());
				return res;
			}

#define searchAndVec1(fname, varname, t) \
	shapefile_db_registry::shapefile_index& shapefile_db_registry::shapefile_index::fname(const t& s) \
			{ \
				varname.insert(boost::lexical_cast<std::string>(s)); \
				return *this; \
			} \
			shapefile_db_registry::shapefile_index& shapefile_db_registry::shapefile_index::fname(const std::vector<t>& s) \
			{ \
				for (const auto &i : s) \
					varname.insert(boost::lexical_cast<std::string>(i)); \
				return *this; \
			}
#define searchAndVec2(fname, varname, t) \
	shapefile_db_registry::shapefile_index& shapefile_db_registry::shapefile_index::fname(const t s) \
			{ \
				varname.insert(boost::lexical_cast<std::string>(s)); \
				return *this; \
			}

			shapefile_db_registry::shapefile_index& shapefile_db_registry::shapefile_index::tag(
				const std::string &a, const std::string &b)
			{
				tags[a] = b; return *this;
			}

			shapefile_db_registry::shapefile_index& shapefile_db_registry::shapefile_index::tag(
				const std::vector<std::pair<std::string, std::string> > &vec)
			{
				for (const auto & v : vec)
					tags[v.first] = v.second;
				return *this;
			}

			shapefile_db_registry::shapefile_index& shapefile_db_registry::shapefile_index::dipoleRange(
				size_t lower, size_t upper)
			{
				dipoleRanges.push_back(std::pair<size_t, size_t>(lower, upper)); return *this;
			}

			shapefile_db_registry::shapefile_index& shapefile_db_registry::shapefile_index::dipoleRange(
				const std::vector<std::pair<size_t, size_t> > &vec)
			{
				for (const auto & v : vec)
					dipoleRanges.push_back(v);
				return *this;
			}

			//searchAndVec1(tag, tags, std::string);
			searchAndVec1(hashLower, hashLowers, std::string);
			searchAndVec1(hashUpper, hashUppers, std::string);
			searchAndVec1(flakeType, flakeTypes, std::string);
			//searchAndVec1(flakeType_uuid, flakeTypeUUIDs, std::string);
			searchAndVec1(flakeRefHashLower, refHashLowers, std::string);
			//searchAndVec1(hash, hashLowers, HASH_t);
			searchAndVec2(hashLower, hashLowers, uint64_t);
			searchAndVec2(hashUpper, hashUppers, uint64_t);

			shapefile_db_registry::shapefile_index& shapefile_db_registry::shapefile_index::standardD(const float d, const float tolpercent)
			{
				standardDs.insert(std::pair<float,float>(d,tolpercent));
				return *this;
			}

#undef searchAndVec1
#undef searchAndVec2

			std::pair<shapefile_db_registry::shapefile_index::collection, std::shared_ptr<rtmath::registry::DBhandler> >
				shapefile_db_registry::shapefile_index::doQuery(std::shared_ptr<rtmath::registry::DBhandler> p, std::shared_ptr<registry::DB_options> o) const
			{
				collection c(new std::set<boost::shared_ptr<shapefile>, shapefile_db_comp >());
				std::shared_ptr<rtmath::registry::DBhandler> fp;

				auto hooks = ::rtmath::registry::usesDLLregistry<shapefile_query_registry, shapefile_db_registry >::getHooks();
				for (const auto &h : *(hooks.get()))
				{
					if (!h.fQuery) continue;
					if (!h.fMatches) continue;
					if (!h.fMatches(p, o)) continue;
					fp = h.fQuery(*this, c, p, o);

					return std::pair < shapefile_db_registry::shapefile_index::collection,
						std::shared_ptr<rtmath::registry::DBhandler> > (c, fp);
				}

				return std::pair<shapefile_db_registry::shapefile_index::collection, 
					std::shared_ptr<rtmath::registry::DBhandler> >
					(c, nullptr);
			}


			std::pair<shapefile_db_registry::shapefile_index::collection, 
				std::shared_ptr<rtmath::registry::DBhandler> >
				shapefile_db_registry::shapefile_index::doQuery(
				collection srcs, bool doUnion, bool doDb,
				std::shared_ptr<rtmath::registry::DBhandler> p, 
				std::shared_ptr<registry::DB_options> o) const
			{
				collection toMergeC(new std::set<boost::shared_ptr<shapefile>, shapefile_db_comp >());
				//collection toMergeS(new std::set<boost::shared_ptr<shapefile>, shapefile_db_comp >());
				collection res(new std::set<boost::shared_ptr<shapefile>, shapefile_db_comp >());
				std::shared_ptr<rtmath::registry::DBhandler> fp;

				std::map<rtmath::HASH_t, boost::shared_ptr<shapefile> > db_hashes; // database results, as a map

				auto hooks = ::rtmath::registry::usesDLLregistry<shapefile_query_registry, shapefile_db_registry >::getHooks();
				if (doDb)
				{
					for (const auto &h : *(hooks.get()))
					{
						if (!h.fQuery) continue;
						if (!h.fMatches) continue;
						if (!h.fMatches(p, o)) continue;
						fp = h.fQuery(*this, toMergeC, p, o);

						//return std::pair < shapefile_db_registry::shapefile_index::collection,
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


				return std::pair<shapefile_db_registry::shapefile_index::collection,
					std::shared_ptr<rtmath::registry::DBhandler> >
					(res, fp);
			}
		}
	}
}

