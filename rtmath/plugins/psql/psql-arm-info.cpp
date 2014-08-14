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
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/macros.h"
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
				using namespace std;
				using std::string;
				std::shared_ptr<psql_handle> h = registry::construct_handle
					<registry::DBhandler, psql_handle>(
					p, PLUGINID, [&](){return std::shared_ptr<psql_handle>(
					new psql_handle(o)); });

				ostringstream squery;
				//squery << "BEGIN;\n";

				string ssel("SELECT site, subsite, product, stream, "
					"datalevel, filename, size, path, startTime, endTime FROM arm_info ");
				bool unionFlag = false;
				// Query is broken up into temporary subtables to accomodate the dates?
				// where discreteTime is within startTime and endTime
				// where the interval s,e contains all or part of startTime and endTime

				if (index.discrete_times.size())
				{
					if (unionFlag) squery << " INTERSECT ";
					squery << ssel << " WHERE ";
					for (auto it = index.discrete_times.begin(); it != index.discrete_times.end(); ++it)
					{
						if (it != index.discrete_times.begin())
							squery << " OR ";
						squery << "'" << *it << "' BETWEEN startTime AND endTime ";
					}
					unionFlag = true;
				}

				if (index.time_ranges.size())
				{
					if (unionFlag) squery << " INTERSECT ";
					squery << ssel << " WHERE ";
					for (auto it = index.time_ranges.begin(); it != index.time_ranges.end(); ++it)
					{
						if (it != index.time_ranges.begin())
							squery << " OR ";
						squery << " tstzrange('" << it->first << "', '" << it->second << "') && tstzrange(startTime, endTime) ";
					}
					unionFlag = true;
				}

				
				if (index.instruments.size() || index.sites.size() ||
					index.subsites.size() || index.data_levels.size() ||
					index.filenames.size() || index.product_names.size() ||
					index.stream_names.size())
				{
					if (unionFlag) squery << " INTERSECT ";
					squery << ssel << " WHERE ";
					unionFlag = true;
				}

				size_t numSel = 0;
				// Select sites, subsites, data_levels, filenames
				auto selString = [&](const std::vector<std::string> &v, const std::string &tblName)
				{
					if (!v.size()) return;
					if (numSel) squery << "AND ";
					size_t numQ = 0;
					squery << tblName << " IN (";
					for (auto it = v.begin(); it != v.end(); ++it)
					{
						if (it != v.begin()) squery << ", ";
						squery << "'" << *it << "'";
						numQ++;
					}
					squery << ") ";
					numSel++;
				};

				selString(index.sites, "site_id");
				selString(index.subsites, "subsite_id");
				selString(index.data_levels, "data_level");
				selString(index.filenames, "filename");
				selString(index.product_names, "product_name");
				selString(index.stream_names, "stream_name");

				squery << ";";

				string s = squery.str();
				//std::cerr << s << std::endl;
				auto resIntersect = h->execute(s.c_str());

				// Turn the result into arm_info objects
				// Unified object is in resIntersect
				for (int i = 0; i < PQntuples(resIntersect.get()); ++i)
				{
					std::shared_ptr<arm_info> ap(new arm_info);
					ap->site = string(PQgetvalue(resIntersect.get(), i, 0));
					ap->subsite = string(PQgetvalue(resIntersect.get(), i, 1));
					ap->product = string(PQgetvalue(resIntersect.get(), i, 2));
					ap->stream = string(PQgetvalue(resIntersect.get(), i, 3));
					ap->datalevel = string(PQgetvalue(resIntersect.get(), i, 4));
					ap->filename = string(PQgetvalue(resIntersect.get(), i, 5));
					ap->filesize = rtmath::macros::m_atoi(PQgetvalue(resIntersect.get(), i, 6));
					ap->filepath = string(PQgetvalue(resIntersect.get(), i, 7));

					string sstime(PQgetvalue(resIntersect.get(), i, 8)), setime(PQgetvalue(resIntersect.get(), i, 9));
					
					ap->startTime = boost::posix_time::time_from_string(sstime);
					ap->endTime = boost::posix_time::time_from_string(setime);

					res->insert(ap);
				}
				
				return h;
			}

			struct subsiteInfo
			{
				std::string site, siteFull;
				std::string subsite, subsiteFull;
				std::string uuid;
				float lat, lon, alt;
			};

			void createBackgroundInfo(std::shared_ptr<psql_handle> h,
				std::map<std::string, subsiteInfo> &subsites, // First is unique "site/subsite" key, 2nd is subsiteInfo (with uuid)
				const std::set<std::string> &products,
				std::map<std::string, std::map<std::string, std::string> > &streams, // product, stream, combo uuid. gets updated with uuids.
				const std::set<std::string> &datalevels
				)
			{
				using namespace std;
				h->sendQuery(
					"BEGIN;"
					"SELECT id FROM site; "
					"SELECT id, subsite, site FROM subsite;"
					"SELECT name FROM product;"
					"SELECT id, name, product FROM stream;"
					"SELECT level FROM datalevel;");
				h->getQueryResult();
				auto hSites = h->getQueryResult();
				auto hSubsites = h->getQueryResult();
				auto hProducts = h->getQueryResult();
				auto hStreams = h->getQueryResult();
				auto hDatalevel = h->getQueryResult();

				using std::string;
				set<string> known_sites;
				map<string, subsiteInfo> known_subsites; // "site/subsite", full subsite info
				set<string> known_products;
				map<string, map<string, string> > known_streams; // first is the instrument, 2nd is the data stream, 3rd is the uuid
				set<string> known_datalevels;


				for (int i = 0; i < PQntuples(hSites.get()); ++i)
					known_sites.emplace(string(PQgetvalue(hSites.get(), i, 0)));
				for (int i = 0; i < PQntuples(hSubsites.get()); ++i) {
					string uuid(PQgetvalue(hSubsites.get(), i, 0));
					string subsite(PQgetvalue(hSubsites.get(), i, 1));
					string site(PQgetvalue(hSubsites.get(), i, 2));
					string key = site; key.append("/"); key.append(subsite);
					subsiteInfo info; info.site = site; info.subsite = subsite;
					info.uuid = uuid;
					known_subsites.emplace(pair<string, subsiteInfo>
						(key, move(info)));
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

				using namespace boost::uuids;
				random_generator gen;
				// Now, insert any missing entries
				for (const auto &s : subsites)
				{
					if (!known_sites.count(s.second.site))
					{
						ss << "insert into site (id) values ('" << s.second.site << "');";
					}
					if (!known_subsites.count(s.first))
					{
						uuid u = gen();
						string suuid = boost::lexical_cast<string>(u);
						subsiteInfo info; info.site = s.second.site; 
						info.subsite = s.second.subsite; info.uuid = suuid;
						info.lat = s.second.lat; info.lon = s.second.lon;
						info.alt = s.second.alt;

						known_subsites[s.first] = info;

						

						ss << "insert into subsite (id, subsite, site, lat, lon, alt, name) values ('"
							<< info.uuid << "', '" << s.second.subsite << "', '" << s.second.site
							<< "', " << s.second.lat
							<< ", " << s.second.lon << ", " << s.second.alt << ", '" 
							<< h->escString(s.second.subsiteFull) << "');";
					}
				}
				subsites = known_subsites;
				for (const auto &s : products)
					if (!known_products.count(s))
						ss << "insert into product (name) values ('" << s << "');";
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
				//cerr << sres << endl;
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

				using std::string;
				for (const auto &i : *c)
				{
					subsiteInfo info;
					info.site = i->site; info.siteFull = i->subsiteFull;
					info.subsite = i->subsite;
					info.subsiteFull = i->subsiteFull;
					info.lat = i->lat; info.lon = i->lon; info.alt = i->alt;

					string sitesubkey = i->site; sitesubkey.append("/"); sitesubkey.append(i->subsite);

					subsites.emplace(std::pair<std::string,
						subsiteInfo >(sitesubkey, std::move(info)));
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
						string sitesubkey = i->site; sitesubkey.append("/"); sitesubkey.append(i->subsite);

						if (!matched_files.count(i->filename))
							sadd << "insert into arm_info_obs (subsite_id, stream_id, datalevel, starttime, endtime, size, filename) values "
								<< "('" << subsites[sitesubkey].uuid << "', '" << streams[i->product].at(i->stream) << "', '" << i->datalevel << "', '"
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
