#include "plugin-psql.h"

/// \brief Provides psql database access
#define _SCL_SECURE_NO_WARNINGS

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <string>
#include <iostream>
#include <sstream>
#include <boost/algorithm/string/trim.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/data/arm_info.h"
#include "../../rtmath/rtmath/data/arm_scanning_radar_sacr.h"
#include "../../rtmath/rtmath/plugin.h"

#include "plugin-psql.h"

namespace rtmath
{
	namespace plugins
	{
		using namespace rtmath::data::arm;
		using std::ostringstream;
		namespace psql
		{
			bool matches(std::shared_ptr<rtmath::registry::DBhandler> p, std::shared_ptr<registry::DB_options>)
			{
				if (!p) return true;
				if (std::string(p->getId()) == std::string(PLUGINID)) return true;
				return false;
			}

			std::shared_ptr<rtmath::registry::DBhandler> search(
				const arm_info_registry::arm_info_index &index,
				arm_info_registry::arm_info_index::collection res,
				std::shared_ptr<rtmath::registry::DBhandler> p, std::shared_ptr<registry::DB_options> o)
			{
				std::shared_ptr<psql_handle> h = registry::construct_handle
					<registry::DBhandler, psql_handle>(
					p, PLUGINID, [&](){return std::shared_ptr<psql_handle>(
					new psql_handle(o)); });

				// Query is broken up into temporary subtables to accomodate the dates?
				// where discreteTime is within startTime and endTime
				// where the interval s,e contains all or part of startTime and endTime

				ostringstream ddates;
				ddates << "CREATE TEMP VIEW SELDDATES AS SELECT * FROM ARM_FILES WHERE ";
				for (auto it = index.discrete_times.begin(); it != index.discrete_times.end(); ++it)
				{
					if (it != index.discrete_times.begin())
						ddates << " OR ";
					ddates << "'" << *it << "' BETWEEN startTime AND endTime ";
				}
				std::string sddates = ddates.str();
				if (index.discrete_times.size())
					h->execute(sddates.c_str());

				ostringstream dranges;
				dranges << "CREATE TEMP VIEW SELRANGES AS SELECT * FROM ARM_FILES WHERE ";
				for (auto it = index.time_ranges.begin(); it != index.time_ranges.end(); ++it)
				{
					if (it != index.time_ranges.begin())
						ddates << " OR ";
					ddates << " tstzrange('" << it->first << "', '" << it->second << "') && tstzrange(startTime, endTime) ";
				}
				std::string sdranges = dranges.str();
				if (index.time_ranges.size())
					h->execute(sdranges.c_str());



				ostringstream sq;
				sq << "CREATE TEMP VIEW SELSTRINGS AS SELECT * FROM ARM_FILES ";
				if (index.instruments.size() || index.sites.size() ||
					index.subsites.size() || index.data_levels.size() ||
					index.filenames.size() || index.product_names.size() ||
					index.stream_names.size()) sq << "WHERE ";

				size_t numSel = 0;
				// Select sites, subsites, data_levels, filenames
				auto selString = [&](const std::vector<std::string> &v, const std::string &tblName)
				{
					if (numSel) sq << "AND ";
					if (v.size())
					{
						sq << tblName << " IN (";
						for (auto it = v.begin(); it != v.end(); ++it)
						{
							if (it != v.begin()) sq << ", ";
							sq << "'" << *it << "'";
						}
						sq << ") ";
						numSel++;
					}
				};

				selString(index.sites, "site_id");
				selString(index.subsites, "subsite_id");
				selString(index.data_levels, "data_level");
				selString(index.filenames, "filename");
				selString(index.product_names, "product_name");
				selString(index.stream_names, "stream_name");

				// Execute the query
				std::string query = sq.str();
				if (numSel)
					h->execute(query.c_str());

				// Unify the results and then drop the tables
				ostringstream inters;
				size_t nInters = 0;
				auto interTbl = [&](const std::string &name)
				{
					if (nInters)
						inters << "INTERSECT";
					inters << " SELECT * FROM " << name;
					nInters++;
				};
				if(numSel)
					interTbl("SELSTRINGS");
				if (index.discrete_times.size())
					interTbl("SELDDATES");
				if (index.time_ranges.size())
					interTbl("SELRANGES");
				std::string sinters = inters.str();
				auto resIntersect = h->execute(sinters.c_str());

				if (numSel)
					h->execute("DROP VIEW SELSTRINGS");
				if (index.discrete_times.size())
					h->execute("DROP VIEW SELDDATES");
				if (index.time_ranges.size())
					h->execute("DROP VIEW SELRANGES");


				// Turn the result into arm_info objects
				// Unified object is in resIntersect
				int nFields = PQnfields(resIntersect.get());
				for (int i = 0; i < nFields; i++)
					std::cerr << PQfname(resIntersect.get(), i) << "\t";
				std::cerr << std::endl << std::endl;

				/* next, print out the rows */
				for (int i = 0; i < PQntuples(resIntersect.get()); i++)
				{
					for (int j = 0; j < nFields; j++)
						std::cerr << PQgetvalue(resIntersect.get(), i, j) << "\t";
					std::cerr << std::endl;
				}
				
				return h;
			}

			struct subsiteInfo
			{
				std::string site;
				float lat, lon, alt;
			};

			void createBackgroundInfo(std::shared_ptr<psql_handle> h,
				const std::map<std::string, subsiteInfo> &subsites, // First is subsite name, 2nd is site
				std::map<std::string, std::string> &products, // first is prod name, 2nd is uuid. Gets filled.
				//std::map<std::string, // stream is harder, as it references uuid, stream name, product_uuid
				const std::set<std::string> &datalevels
				)
			{
				auto hSites = h->execute("SELECT id FROM site;");
				auto hSubsites = h->execute("SELECT id, site FROM subsite;");
				auto hProducts = h->execute("SELECT id, name FROM product;");
				auto hStreams = h->execute("SELECT id, name, product FROM stream;");
				auto hDatalevel = h->execute("SELECT level FROM datalevel;");

				std::set<std::string> known_sites;
				std::map<std::string, std::string> known_subsites;
				std::map<std::string, std::string> known_products;
				// known_streams
				std::set<std::string> known_datalevels;


				for (int i = 0; i < PQntuples(hSites.get()); ++i)
					known_sites.emplace(std::string(PQgetvalue(hSites.get(), i, 0)));
				for (int i = 0; i < PQntuples(hSubsites.get()); ++i) {
					known_subsites.emplace(std::pair<std::string, std::string>
						(std::string(PQgetvalue(hSubsites.get(), i, 0)),
						std::string(PQgetvalue(hSubsites.get(), i, 1))));
				}
				// todo: add streams
				for (int i = 0; i < PQntuples(hDatalevel.get()); ++i)
					known_datalevels.emplace(std::string(PQgetvalue(hDatalevel.get(), i, 0)));
			}

			std::shared_ptr<rtmath::registry::DBhandler> update(
				const arm_info_registry::arm_info_index::collection c,
				arm_info_registry::updateType t,
				std::shared_ptr<rtmath::registry::DBhandler> p, std::shared_ptr<registry::DB_options> o)
			{
				std::shared_ptr<psql_handle> h = registry::construct_handle
					<registry::DBhandler, psql_handle>(
					p, PLUGINID, [&](){return std::shared_ptr<psql_handle>(
					new psql_handle(o)); });

				if (t == arm_info_registry::updateType::INSERT_ONLY)
				{
					// Search site, subsite, datalevel, product_name and add if missing
					// Just select all from these tables
					std::map<std::string, subsiteInfo > subsites;
					std::map<std::string, std::string> products;
					std::set<std::string> datalevels;

					for (const auto &i : *c)
					{
						subsiteInfo info;
						info.site = i->site;
						info.lat = i->lat; info.lon = i->lon; info.alt = i->alt;
						subsites.emplace(std::pair<std::string, 
							subsiteInfo >(i->subsite, std::move(info)));
						products.emplace(std::pair<std::string, std::string>(i->product, ""));
						datalevels.emplace(i->datalevel);
					}
					
					createBackgroundInfo(h, subsites, products, datalevels);

				}

				/*
				// First, look for any matching filenames.
				arm_info_registry::arm_info_index::collection 
					toUpdate = arm_info::makeCollection(),
					toInsert = arm_info::makeCollection();

				std::vector<const char*> filenames;
				filenames.reserve(c->size());
				for (const auto &i : *c)
					filenames.push_back(i->filename.c_str());

				//auto res = 
				*/


				return h;
			}
		}
	}
}
