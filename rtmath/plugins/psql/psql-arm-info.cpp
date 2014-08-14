#pragma warning( disable : 4996 ) // -D_SCL_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS
#include "plugin-psql.h"

/// \brief Provides psql database access


#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <string>
#include <iostream>
#include <sstream>
#include <boost/algorithm/string/trim.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

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
				if (numSel)
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
				const std::set<std::string> &products,
				std::map<std::string, std::map<std::string, std::string> > &streams, // product, stream, combo uuid. gets updated with uuids.
				const std::set<std::string> &datalevels
				)
			{
				using namespace std;
				h->sendQuery(
					"BEGIN;"
					"SELECT id FROM site; "
					"SELECT id, site FROM subsite;"
					"SELECT name FROM product;"
					"SELECT id, name, product FROM stream;"
					"SELECT level FROM datalevel;");
				h->getQueryResult();
				auto hSites = h->getQueryResult();
				auto hSubsites = h->getQueryResult();
				auto hProducts = h->getQueryResult();
				auto hStreams = h->getQueryResult();
				auto hDatalevel = h->getQueryResult();

				set<string> known_sites;
				map<string, string> known_subsites;
				set<string> known_products;
				map<string, map<string, string> > known_streams; // first is the instrument, 2nd is the data stream, 3rd is the uuid
				set<string> known_datalevels;


				for (int i = 0; i < PQntuples(hSites.get()); ++i)
					known_sites.emplace(string(PQgetvalue(hSites.get(), i, 0)));
				for (int i = 0; i < PQntuples(hSubsites.get()); ++i) {
					known_subsites.emplace(pair<string, string>
						(string(PQgetvalue(hSubsites.get(), i, 0)),
						string(PQgetvalue(hSubsites.get(), i, 1))));
				}
				for (int i = 0; i < PQntuples(hProducts.get()); ++i)
					known_products.emplace(string(PQgetvalue(hProducts.get(), i, 0)));
				for (int i = 0; i < PQntuples(hStreams.get()); ++i) {
					string product(PQgetvalue(hStreams.get(), i, 2));
					if (!known_streams.count(product))
						known_streams[product] = move(map<string, string>());

					known_streams[product].emplace(pair<string, string>
						(string(PQgetvalue(hStreams.get(), i, 1)),
						string(PQgetvalue(hStreams.get(), i, 0))));
				}
				for (int i = 0; i < PQntuples(hDatalevel.get()); ++i)
					known_datalevels.emplace(string(PQgetvalue(hDatalevel.get(), i, 0)));

				ostringstream ss;

				// Now, insert any missing entries
				for (const auto &s : subsites)
				{
					if (!known_sites.count(s.second.site))
					{
						ss << "insert into site (id) values ('" << s.second.site << "');";
					}
					if (!known_subsites.count(s.first))
					{
						ss << "insert into subsite (id, site, lat, lon, alt) values ('"
							<< s.first << "', '" << s.second.site << "', " << s.second.lat
							<< ", " << s.second.lon << ", " << s.second.alt << ");";
					}
				}
				for (const auto &s : products)
					if (!known_products.count(s))
						ss << "insert into product (name) values ('" << s << "');";
				using namespace boost::uuids;
				random_generator gen;
				for (const auto &s : streams)
				{
					string product(s.first);
					if (!known_streams.count(product))
						known_streams[product] = move(map<string, string>());

					for (const auto &sb : s.second)
					{
						// Generate a uuid
						uuid u = gen();
						string suuid = boost::lexical_cast<string>(u);
						if (!known_streams[product].count(sb.first))
						{
							known_streams[product].insert(pair < string, string >
								(sb.first, suuid));
							ss << "insert into stream (id, name, product) values ('" << suuid << "', '"
								<< sb.first << "', '" << product << "');";
						}
					}
				}
				streams = known_streams; // For object additions
				for (const auto &s : datalevels)
				{
					if (!known_datalevels.count(s))
					{
						ss << "insert into datalevel (level) values ('" << s << "');";
					}
				}

				ss << "COMMIT;";
				string sres = ss.str();
				h->execute(sres.c_str());
			}

			std::shared_ptr<rtmath::registry::DBhandler> update(
				const arm_info_registry::arm_info_index::collection c,
				arm_info_registry::updateType t,
				std::shared_ptr<rtmath::registry::DBhandler> p, std::shared_ptr<registry::DB_options> o)
			{
				using namespace std;
				std::shared_ptr<psql_handle> h = registry::construct_handle
					<registry::DBhandler, psql_handle>(
					p, PLUGINID, [&](){return std::shared_ptr<psql_handle>(
					new psql_handle(o)); });

				

				// Search site, subsite, datalevel, product_name and add if missing
				// Just select all from these tables
				std::map<std::string, subsiteInfo > subsites;
				std::set<std::string> products;
				std::map<std::string, std::map<std::string, std::string> > streams;
				std::set<std::string> datalevels;

				for (const auto &i : *c)
				{
					subsiteInfo info;
					info.site = i->site;
					info.lat = i->lat; info.lon = i->lon; info.alt = i->alt;
					subsites.emplace(std::pair<std::string,
						subsiteInfo >(i->subsite, std::move(info)));
					products.emplace(i->product);
					if (!streams.count(i->product)) streams[i->product] = std::move(std::map<std::string, std::string>());
					streams[i->product].emplace(std::pair<std::string, std::string>(i->stream, ""));
					datalevels.emplace(i->datalevel);
				}

				createBackgroundInfo(h, subsites, products, streams, datalevels);

				// Actually insert the data
				h->execute("BEGIN;");
				std::ostringstream sadd;

				// Query the database regarding matching filenames (determines insert / update operation)
				set<string> matched_files;
				{
					ostringstream qmatch;
					qmatch << "select filename from arm_info_obs intersect values ";
					for (auto it = c->begin(); it != c->end(); ++it)
					{
						if (it != c->begin()) qmatch << ", ";
						qmatch << "('" << (*it)->filename << "')";
					}
					qmatch << ";";
					string sqm = qmatch.str();
					auto hMatches = h->execute(sqm.c_str());

					for (int i = 0; i < PQntuples(hMatches.get()); ++i)
						matched_files.emplace(string(PQgetvalue(hMatches.get(), i, 0)));
				}

				for (const auto &i : *c)
				{
					if (t != arm_info_registry::updateType::UPDATE_ONLY)
					{
						if (!matched_files.count(i->filename))
							sadd << "insert into arm_info_obs (subsite, stream_id, datalevel, starttime, endtime, size, filename) values "
								<< "('" << i->subsite << "', '" << streams[i->product].at(i->stream) << "', '" << i->datalevel << "', '"
								<< i->startTime << "', '" << i->endTime << "', " << i->filesize << ", '" << i->filename << "');";
					}
				}

				sadd << "COMMIT;";
				std::string sres = sadd.str();
				h->execute(sres.c_str());

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
