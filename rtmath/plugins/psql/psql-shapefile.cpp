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
					boost::shared_ptr<shapefile> ap(new shapefile);

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

			void createBackgroundInfo(std::shared_ptr<psql_handle> h,
				std::map<std::string, std::string > &flakeTypes
				)
			{
				using namespace std;
				h->sendQuery(
					"BEGIN;"
					"SELECT id, name FROM flakeTypes; ");
				h->getQueryResult();
				auto hFlakeTypes = h->getQueryResult();
				
				using std::string;
				map<string, string> known_flaketypes;

				for (int i = 0; i < PQntuples(hFlakeTypes.get()); ++i)
					known_flaketypes[string(PQgetvalue(hFlakeTypes.get(), i, 1))] 
					= string(PQgetvalue(hFlakeTypes.get(), i, 0));
				
				ostringstream ss;

				using namespace boost::uuids;
				random_generator gen;
				// Now, insert any missing entries
				for (const auto &s : flakeTypes)
				{
					string ftype(s.first);
					if (!known_flaketypes.count(ftype))
					{
						// Generate a uuid
						uuid u = gen();
						string suuid = boost::lexical_cast<string>(u);
						known_flaketypes.insert(pair < string, string >
							(ftype, suuid));
						ss << "insert into flakeTypes (id, name) values ('" << suuid << "', '"
							<< ftype << "');";
					}
				}
				flakeTypes = known_flaketypes; // For object additions
				
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

				
				map<string, string> flakeTypes;
				for (const auto &i : *c)
				{
					for (const auto &t : i->tags)
					{
						if (t.first == "flake_type")
						{
							flakeTypes[t.second] = "";
						}
					}
				}
				using std::string;
				/*
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
				*/

				createBackgroundInfo(h, flakeTypes);

				// Actually insert the data
				h->execute("BEGIN;");
				std::ostringstream sadd;

				// Query the database regarding matching filenames (determines insert / update operation)
				set<string> matched_hashes;
				{
					ostringstream qmatch;
					qmatch << "select hashLower from flake intersect values ";
					for (auto it = c->begin(); it != c->end(); ++it)
					{
						if (it != c->begin()) qmatch << ", ";
						qmatch << "('" << (*it)->hash().lower << "')";
					}
					qmatch << ";";
					string sqm = qmatch.str();
					auto hMatches = h->execute(sqm.c_str());

					for (int i = 0; i < PQntuples(hMatches.get()); ++i)
						matched_hashes.emplace(string(PQgetvalue(hMatches.get(), i, 0)));
				}

				for (const auto &i : *c)
				{
					// Determine flakeType_id
					string flakeType_id;
					// Determine tag list
					string tags;
					// Determine flake reference_id
					string refId;
					auto genTagList = [&](std::string &tagout, std::string &flakeTypeid, std::string &refId)
					{
						ostringstream stags;
						stags << "{";
						bool firstTag = false;
						for (const auto &t : i->tags)
						{
							if (t.first == "flake_reference")
								refId = t.second;
							else if (t.first == "flake_type")
							{
								flakeType_id = flakeTypes.at(t.second);
							} else {
								if (firstTag) stags << ", ";
								stags << "\"" << t.first << "=" << t.second << "\"";
								firstTag = true;
							}

						}
						stags << "}";
						tagout = stags.str();
					};
					genTagList(tags, flakeType_id, refId);
					

					if (!matched_hashes.count(i->hash().string()))
					{
						if (t != shapefile_db_registry::updateType::UPDATE_ONLY)
						{
							sadd << "insert into flake (hashLower, hashUpper";
							if (flakeType_id.size())
								sadd << ", flakeType_id";
							if (i->tags.size())
								sadd << ", tags";
							if (i->standardD)
								sadd << ", standardD";
							if (i->desc.size())
								sadd << ", description";
							if (refId.size())
								sadd << ", flake_references";
							sadd << ") values ('" << i->hash().lower << "', '" << i->hash().upper << "'";
							if (flakeType_id.size())
								sadd << ", '" << flakeType_id << "'";
							if (i->tags.size())
								sadd << ", '" << tags << "'";
							if (i->standardD)
								sadd << ", " << i->standardD;
							if (i->desc.size())
								sadd << ", '" << i->desc << "'";
							if (refId.size())
								sadd << ", '" << refId << "'";
							sadd << ");";
							
						}
					}
					// TODO: add update code
				}

				sadd << "COMMIT;";
				std::string sres = sadd.str();
				h->execute(sres.c_str());


				return h;
			}
		}
	}
}
