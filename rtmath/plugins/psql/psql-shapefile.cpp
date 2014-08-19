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
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/macros.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/plugin.h"

#include "plugin-psql.h"

namespace rtmath
{
	namespace plugins
	{
		using namespace rtmath::ddscat::shapefile;
		using std::ostringstream;
		namespace psql
		{

			std::shared_ptr<rtmath::registry::DBhandler> searchSHP(
				const rtmath::ddscat::shapefile::shapefile_db_registry::shapefile_index &index,
				rtmath::ddscat::shapefile::shapefile_db_registry::shapefile_index::collection res,
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

				string ssel("SELECT hashLower, hashUpper, flakeType_id, tags, "
					"standardD, description, flake_references FROM flake ");
				bool unionFlag = false;
				
				// Dipole spacing ranges
				if (index.standardDs.size())
				{
					if (unionFlag) squery << " INTERSECT ";
					squery << ssel << " WHERE ";
					for (auto it = index.standardDs.begin(); it != index.standardDs.end(); ++it)
					{
						if (it != index.standardDs.begin())
							squery << " OR ";
						squery << "numrange( " << it->first * (1.0f - it->second) << " , "
							<< it->first * (1.0f + it->second) << " ) @> standardD ";
					}
					unionFlag = true;
				}

				if (index.tags.size() || index.hashLowers.size() ||
					index.hashUppers.size() || index.flakeTypes.size() ||
					index.flakeTypeUUIDs.size() || index.refHashLowers.size())
				{
					if (unionFlag) squery << " INTERSECT ";
					squery << ssel << " WHERE ";
					unionFlag = true;
				}

				size_t numSel = 0;
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

				selString(index.tags, "tag");
				selString(index.hashLowers, "hashLower");
				selString(index.hashUppers, "hashUpper");
				selString(index.flakeTypes, "flakeType");
				selString(index.flakeTypeUUIDs, "flakeType_UUID");
				selString(index.refHashLowers, "flake_references");

				squery << ";";

				string s = squery.str();
				//std::cerr << s << std::endl;
				auto resIntersect = h->execute(s.c_str());

				// Turn the result into shapefile stub objects
				// Unified object is in resIntersect
				// SELECT hashLower, hashUpper, flakeType_id, tags, "
					//"standardD, description, flake_references FROM flake 
				for (int i = 0; i < PQntuples(resIntersect.get()); ++i)
				{
					std::shared_ptr<shapefile> ap(new shapefile);

					ap->standardD = rtmath::macros::m_atof<float>(PQgetvalue(resIntersect.get(), i, 4));
					ap->setHash(HASH_t( rtmath::macros::m_atoi<uint64_t>(PQgetvalue(resIntersect.get(), i, 0)),
						rtmath::macros::m_atoi<uint64_t>(PQgetvalue(resIntersect.get(), i, 1))));

					ap->desc = string(PQgetvalue(resIntersect.get(), i, 5));

					ap->tags.insert(std::pair<std::string,std::string>("flake_reference",
						std::string(PQgetvalue(resIntersect.get(), i, 6))));
					ap->tags.insert(std::pair<std::string,std::string>("flake_type",
						std::string(PQgetvalue(resIntersect.get(), i, 2))));

					// Iterate over all tags and add to the tags field.
					// psql tags saved as key=value strings.
					string othertags(PQgetvalue(resIntersect.get(), i, 3));
					// Parse tags (they look like {a=b,c=d,e=f})
					// First, remove { and }
					othertags.erase(std::remove(othertags.begin(), othertags.end(), '{'), othertags.end());
					othertags.erase(std::remove(othertags.begin(), othertags.end(), '}'), othertags.end());
					// Split based on commas
					std::vector<std::string> vother;
					rtmath::config::splitVector(othertags, vother, ',');
					// Insert, splitting on =
					for (const auto & i : vother)
					{
						vector<string> vs;
						rtmath::config::splitVector(i, vs, '=');
						string a, b;
						if (vs.size()) a = vs[0];
						if (vs.size() > 1) b = vs[1];
						ap->tags.insert(std::pair<std::string,std::string>(a,b));
					}

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

			std::shared_ptr<rtmath::registry::DBhandler> updateSHP(
				const rtmath::ddscat::shapefile::shapefile_db_registry::shapefile_index::collection c,
				rtmath::ddscat::shapefile::shapefile_db_registry::updateType t,
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


				return h;
			}
		}
	}
}
