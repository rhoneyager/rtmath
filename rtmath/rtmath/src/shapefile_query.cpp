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
				varname.push_back(boost::lexical_cast<std::string>(s)); \
				return *this; \
			} \
			shapefile_db_registry::shapefile_index& shapefile_db_registry::shapefile_index::fname(const std::vector<t>& s) \
			{ \
				std::vector<std::string> res(s.size()); \
				for (const auto i : s) \
					res.push_back(boost::lexical_cast<std::string>(i)); \
				varname.insert(varname.begin(), s.begin(), s.end()); \
				return *this; \
			}
#define searchAndVec2(fname, varname, t) \
	shapefile_db_registry::shapefile_index& shapefile_db_registry::shapefile_index::fname(const t s) \
			{ \
				varname.push_back(boost::lexical_cast<std::string>(s)); \
				return *this; \
			}

			searchAndVec1(tag, tags, std::string);
			searchAndVec1(hashLower, hashLowers, std::string);
			searchAndVec1(hashUpper, hashUppers, std::string);
			searchAndVec1(flakeType, flakeTypes, std::string);
			searchAndVec1(flakeType_uuid, flakeTypeUUIDs, std::string);
			searchAndVec1(flakeRefHashLower, refHashLowers, std::string);
			//searchAndVec1(hash, hashLowers, HASH_t);
			searchAndVec2(hashLower, hashLowers, uint64_t);
			searchAndVec2(hashUpper, hashUppers, uint64_t);

			shapefile_db_registry::shapefile_index& shapefile_db_registry::shapefile_index::standardD(const float d, const float tolpercent)
			{
				standardDs.push_back(std::pair<float,float>(d,tolpercent));
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

		}
	}
}

