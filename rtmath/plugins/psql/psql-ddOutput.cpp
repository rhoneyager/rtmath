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
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/plugin.h"

#include "plugin-psql.h"

namespace rtmath
{
	namespace plugins
	{
		using namespace rtmath::ddscat;
		using std::ostringstream;
		namespace psql
		{
			/*
			void createBackgroundInfo(std::shared_ptr<psql_handle> h,
				std::map<std::string, std::string > &flakeTypes, // type, uuid
				std::map<std::string, std::string > &flakeRevTypes // uuid, type (used in searchSHP)
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

				for (const auto &s : flakeTypes)
					flakeRevTypes[s.second] = s.first;

				ss << "COMMIT;";
				string sres = ss.str();
				//cerr << sres << endl;
				h->execute(sres.c_str());
			}

			*/

			template<class T> void makeStrNumRange(std::ostringstream &squery,
				const std::string &paramName,
				const T lower, const T upper)
			{
				throw debug::xUpcast("psql", "makeStrNumRange");
			}

			template<> void makeStrNumRange<float>(std::ostringstream &squery,
				const std::string &paramName,
				const float lower, const float upper)
			{
				squery << "cast(" << paramName << " as numeric ) <@ numrange( " << lower << " , "
					<< upper << " ) ";
			}

			template<> void makeStrNumRange<double>(std::ostringstream &squery,
				const std::string &paramName,
				const double lower, const double upper)
			{
				squery << "cast(" << paramName << " as numeric ) <@ numrange( " << lower << " , "
					<< upper << " ) ";
			}

			template<> void makeStrNumRange<int>(std::ostringstream &squery,
				const std::string &paramName,
				const int lower, const int upper)
			{
				squery << paramName << " <@ int4range( " << lower << " , "
					<< upper << " ) ";
			}

			template<> void makeStrNumRange<unsigned int>(std::ostringstream &squery,
				const std::string &paramName,
				const unsigned int lower, const unsigned int upper)
			{
				squery << paramName << " <@ int4range( " << lower << " , "
					<< upper << " ) ";
			}

			template<> void makeStrNumRange<unsigned long>(std::ostringstream &squery,
				const std::string &paramName,
				const unsigned long lower, const unsigned long upper)
			{
				squery << paramName << " <@ int4range( " << lower << " , "
					<< upper << " ) ";
			}

			template<> void makeStrNumRange<long>(std::ostringstream &squery,
				const std::string &paramName,
				const long lower, const long upper)
			{
				squery << paramName << " <@ int4range( " << lower << " , "
					<< upper << " ) ";
			}

			template<> void makeStrNumRange<unsigned long long>(std::ostringstream &squery,
				const std::string &paramName,
				const unsigned long long lower, const unsigned long long upper)
			{
				squery << paramName << " <@ int4range( " << lower << " , "
					<< upper << " ) ";
			}


			template<class T>
			void constructSearchInterval(std::ostringstream &squery, 
				const std::string &ssel,
				const std::string &sclose,
				const std::string &paramName,
				const rtmath::config::intervals<T> &vals,
				bool &unionFlag, bool allowNull = false)
			{
				if (!vals.ranges.size()) return;
				if (unionFlag) squery << " INTERSECT ";
				squery << ssel << " WHERE " << paramName << " ";
				if (allowNull) squery << "is NULL ";
				for (auto it = vals.ranges.begin(); it != vals.ranges.end(); ++it)
				{
					if (it != vals.ranges.begin() || allowNull)
						squery << " OR ";
					makeStrNumRange<T>(squery, paramName, it->first, it->second);
				}
				squery << sclose;
				unionFlag = true;
			}

			std::shared_ptr<rtmath::registry::DBhandler> searchRUN(
				const rtmath::ddscat::ddOutput_db_registry::ddOutput_index &index,
				rtmath::ddscat::ddOutput_db_registry::ddOutput_index::collection res,
				std::shared_ptr<rtmath::registry::DBhandler> p, std::shared_ptr<registry::DB_options> o)
			{
				using namespace std;
				using std::string;
				std::shared_ptr<psql_handle> h = registry::construct_handle
					<registry::DBhandler, psql_handle>(
					p, PLUGINID, [&](){return std::shared_ptr<psql_handle>(
					new psql_handle(o)); });


				//map<string, string> flakeTypes, flakeRevTypes;
				//createBackgroundInfo(h, flakeTypes, flakeRevTypes); // Populate the flaketype name / uuid map

				ostringstream squery;
				//squery << "BEGIN;\n";

				string ssel("SELECT flakeResult.hashLower, flakeResult.runid, flakeResult.resultid, flakeResult.aeff, "
					"flakeType_id, flakeTypes.name, flake.numDipoles, flake.standardD, "
					"flakeRuns.frequency, flakeRuns.temperature, flakeRuns.nBetas, flakeRuns.nThetas, "
					"flakeRuns.nPhis, flakeRuns.polarization, flakeResult.path "
					"FROM flakeResult, flakeRuns, flakeTypes, flake ");
				string sclose(
					"where flake.hashLower = flakeResult.hashLower "
					"and flake.flakeType_id = flakeTypes.id "
					"and flakeResult.runid = flakeRuns.id "
					"order by flakeRuns.frequency, flakeRuns.temperature, flakeResult.aeff"
					);
				bool unionFlag = false;

				// Dipole spacing ranges
				constructSearchInterval<float>(squery, ssel, sclose, "flake.standardD", index.dipoleSpacings, unionFlag, true);
				// Number of dipoles
				constructSearchInterval<size_t>(squery, ssel, sclose, "flake.numDipoles", index.dipoleNumbers, unionFlag, true);
				// Rotations
				constructSearchInterval<size_t>(squery, ssel, sclose, "flakeRuns.nBetas", index.betaRanges, unionFlag, false);
				constructSearchInterval<size_t>(squery, ssel, sclose, "flakeRuns.nThetas", index.thetaRanges, unionFlag, false);
				constructSearchInterval<size_t>(squery, ssel, sclose, "flakeRuns.nPhis", index.phiRanges, unionFlag, false);
				// Freq, aeff, temp
				constructSearchInterval<float>(squery, ssel, sclose, "flakeRuns.frequency", index.freqRanges, unionFlag, false);
				constructSearchInterval<float>(squery, ssel, sclose, "flakeResult.aeff", index.aeffRanges, unionFlag, false);
				constructSearchInterval<float>(squery, ssel, sclose, "flakeRuns.temperature", index.tempRanges, unionFlag, false);


				if (//index.tags.size() || 
					index.hashLowers.size() ||
					index.hashUppers.size() || 
					index.flakeTypes.size() // ||
					//index.refHashLowers.size()
					)
				{
					if (unionFlag) squery << " INTERSECT ";
					squery << ssel << " WHERE ";
					unionFlag = true;
				}

				size_t numSel = 0;
				auto selString = [&](const std::set<std::string> &v, const std::string &tblName)
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
				auto selStringMap = [&](const std::set<std::string> &v, const std::string &tblName, const std::map<std::string, std::string> &m)
				{
					if (!v.size()) return;
					if (numSel) squery << "AND ";
					size_t numQ = 0;
					squery << tblName << " IN (";
					for (auto it = v.begin(); it != v.end(); ++it)
					{
						if (m.count(*it))
						{
							if (it != v.begin()) squery << ", ";
							squery << "'" << m.at(*it) << "'";
							numQ++;
						}
						else RTthrow(debug::xMissingHash())
							<< debug::hash(*it);
					}
					squery << ") ";
					numSel++;
				};
				auto selStringArray = [&](const std::map<std::string, std::string> &v, const std::string &tblName)
				{
					if (!v.size()) return;
					for (auto it = v.begin(); it != v.end(); ++it)
					{
						if (numSel) squery << "AND ";
						squery << "'" << it->first << "=" << it->second << "' = any(" << tblName << ") ";
						numSel++;
					}
				};

				//selStringArray(index.tags, "tags");
				selString(index.hashLowers, "flakeResult.hashLower");
				selString(index.hashUppers, "flakeResult.hashUpper");

				selString(index.runids, "flakeResult.resultid");
				selString(index.flakeTypes, "flakeTypes.name"); // , flakeTypes);
				selString(index.pol, "flakeRuns.polarization");
				//selString(index.refHashLowers, "flake_references");

				squery << ";";

				string s = squery.str();
				//std::cerr << s << std::endl;
				auto resIntersect = h->execute(s.c_str());

				/*
				"SELECT flakeResult.hashLower, flakeResult.runid, flakeResult.resultid, flakeResult.aeff, "
				"flakeType_id, flakeTypes.name, flake.numDipoles, flake.standardD, "
				"flakeRuns.frequency, flakeRuns.temperature, flakeRuns.nBetas, flakeRuns.nThetas, "
				"flakeRuns.nPhis, flakeRuns.polarization, flakeResult.path "
				"FROM flakeResult, flakeRuns, flakeTypes, flake "
				*/
				for (int i = 0; i < PQntuples(resIntersect.get()); ++i)
				{
					HASH_t shpLower(rtmath::macros::m_atoi<uint64_t>(PQgetvalue(resIntersect.get(), i, 0)), 0);
					HASH_t resultidLower(rtmath::macros::m_atoi<uint64_t>(PQgetvalue(resIntersect.get(), i, 2)), 0);
					std::string pathOverride(PQgetvalue(resIntersect.get(), i, 14));
					//boost::shared_ptr<ddOutput> ap = ddOutput::

					boost::shared_ptr<ddOutput> ap;
					if (pathOverride.size()) ap = ddOutput::generate(pathOverride);
					else ap = ddOutput::loadHash(shpLower.string(), resultidLower.string());

					res->insert(ap);
				}

				return h;
			}


			std::shared_ptr<rtmath::registry::DBhandler> updateRUN(
				const rtmath::ddscat::ddOutput_db_registry::ddOutput_index::collection c,
				rtmath::ddscat::ddOutput_db_registry::updateType t,
				std::shared_ptr<rtmath::registry::DBhandler> p, std::shared_ptr<registry::DB_options> o)
			{
				using namespace std;
				std::shared_ptr<psql_handle> h = registry::construct_handle
					<registry::DBhandler, psql_handle>(
					p, PLUGINID, [&](){return std::shared_ptr<psql_handle>(
					new psql_handle(o)); });


				using std::string;

				/// \todo Add base search function to pull in the flake run database, with uuids
				/// \todo Add a matching function that finds the run uuid for a given run
				//createBackgroundInfo(h, flakeTypes, flakeRevTypes);

				// Actually insert the data
				h->execute("BEGIN;");
				std::ostringstream sadd;

				// If a ddOutput run already has the matching tag that matches to a run, then use it.
				// Otherwise, find the run id. This is accomplished by looking at the existing runs, and 
				// matching based on 1) the flake type, 2) near-freq, 3) near-temp, 4) polarization,
				// 5) number of rotations (beta, theta, phi), 6) ddscat version, 7) completion flag (remove 
				// cancelled and not started), 8) decimation, 9) perturbation.

				for (const auto &i : *c)
				{
					// Source file path
					string srcpath; // currently unused
					// Timestamp for import
					string tsadded; // currently unused

					// Determine tag list
					string tags; // mostly unused
					// Determine run id
					string run_id;
					auto genTagList = [&](std::string &tagout, std::string &run_id)
					{
						ostringstream stags;
						stags << "{";
						bool firstTag = false;
						for (const auto &t : i->tags)
						{
							if (t.first == "run_id")
								run_id = t.second;
							else {
								if (firstTag) stags << ", ";
								stags << "\"" << t.first << "=" << t.second << "\"";
								firstTag = true;
							}

						}
						stags << "}";
						tagout = stags.str();
					};
					genTagList(tags, run_id);


					//if (!matched_hashes.count(i->hash().string()))
					if (1)
					{
						if (t != ddOutput_db_registry::updateType::UPDATE_ONLY)
						{
							sadd << "insert into flakeResult (hashLower, runid, aeff, ";
							if (i->avgdata.hasAvg)
								sadd << "Qsca_iso, Qbk_iso, Qext_iso, Qabs_iso, g_iso";
							if (srcpath.size())
								sadd << ", path";
							if (tsadded.size())
								sadd << ", tsAdded";
							sadd << ") values ('" << i->shapeHash.lower << "', '" << run_id << "', " << i->aeff;
							if (i->avgdata.hasAvg)
							{
								sadd << ", " << i->avgdata.avg(0, rtmath::ddscat::ddOutput::stat_entries::QSCAM)
									<< ", " << i->avgdata.avg(0, rtmath::ddscat::ddOutput::stat_entries::QBKM)
									<< ", " << i->avgdata.avg(0, rtmath::ddscat::ddOutput::stat_entries::QEXTM)
									<< ", " << i->avgdata.avg(0, rtmath::ddscat::ddOutput::stat_entries::QABSM)
									<< ", " << i->avgdata.avg(0, rtmath::ddscat::ddOutput::stat_entries::G1M);
							}
							if (srcpath.size())
								sadd << ", 'path'";
							if (tsadded.size())
								sadd << ", '" << tsadded << "'";
							
							sadd << ");";
						}
					} else {
						if (t != ddOutput_db_registry::updateType::INSERT_ONLY)
						{
							sadd << "update flakeResult set ";
							bool needComma = false;
							if (i->aeff)
							{
								if (needComma) sadd << ", ";
								sadd << "aeff = " << i->aeff;
								needComma = true;
							}
							if (srcpath.size())
							{
								if (needComma) sadd << ", ";
								sadd << "path = '" << srcpath << "' ";
								needComma = true;
							}
							{
								if (needComma) sadd << ", ";
								sadd << "tsAdded = '" << tsadded << "' ";
								needComma = true;
							}
							{
								if (needComma) sadd << ", ";
								sadd << "Qsca_iso = " << i->avgdata.avg(0, rtmath::ddscat::ddOutput::stat_entries::QSCAM);
								needComma = true;
							}
							{
								if (needComma) sadd << ", ";
								sadd << "Qbk_iso = " << i->avgdata.avg(0, rtmath::ddscat::ddOutput::stat_entries::QBKM);
								needComma = true;
							}
							{
								if (needComma) sadd << ", ";
								sadd << "Qext_iso = " << i->avgdata.avg(0, rtmath::ddscat::ddOutput::stat_entries::QEXTM);
								needComma = true;
							}
							{
								if (needComma) sadd << ", ";
								sadd << "Qabs_iso = " << i->avgdata.avg(0, rtmath::ddscat::ddOutput::stat_entries::QABSM);
								needComma = true;
							}
							{
								if (needComma) sadd << ", ";
								sadd << "g_iso = " << i->avgdata.avg(0, rtmath::ddscat::ddOutput::stat_entries::G1M);
								needComma = true;
							}
							sadd << " where hashlower = '" << i->shapeHash.lower << "'"
								<< " AND runid = '" << run_id << "';";
						}
					}
				}

				sadd << "COMMIT;";
				std::string sres = sadd.str();
				h->execute(sres.c_str());


				return h;
			}

			/*
			std::shared_ptr<rtmath::registry::DBhandler> updateRUN(
				const rtmath::ddscat::ddOutput_db_registry::ddOutput_index::collection c,
				rtmath::ddscat::ddOutput_db_registry::updateType t,
				std::shared_ptr<rtmath::registry::DBhandler> p, std::shared_ptr<registry::DB_options> o)
			{
				using namespace std;
				std::shared_ptr<psql_handle> h = registry::construct_handle
					<registry::DBhandler, psql_handle>(
					p, PLUGINID, [&](){return std::shared_ptr<psql_handle>(
					new psql_handle(o)); });


				map<string, string> flakeTypes, flakeRevTypes;
				for (const auto &i : *c)
				{
					for (const auto &t : i->tags)
					{
						if (t.first == "flake_classification")
						{
							flakeTypes[t.second] = "";
						}
					}
				}
				using std::string;

				createBackgroundInfo(h, flakeTypes, flakeRevTypes);

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
							else if (t.first == "flake_classification")
							{
								flakeType_id = flakeTypes.at(t.second);
							}
							else {
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
							sadd << "insert into flake (hashLower, hashUpper, numDipoles";
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
							sadd << ") values ('" << i->hash().lower << "', '" << i->hash().upper << "'"
								<< ", " << i->numPoints;
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
					else {
						if (t != shapefile_db_registry::updateType::UPDATE_ONLY)
						{
							sadd << "update flake set ";
							bool needComma = false;
							if (flakeType_id.size())
							{
								if (needComma) sadd << ", ";
								sadd << "flakeType_id = '" << flakeType_id << "' ";
								needComma = true;
							}
							if (i->tags.size())
							{
								if (needComma) sadd << ", ";
								sadd << "tags = '" << tags << "' ";
								needComma = true;
							}
							if (i->standardD)
							{
								if (needComma) sadd << ", ";
								sadd << "standardD = " << i->standardD;
								needComma = true;
							}
							if (i->numPoints)
							{
								if (needComma) sadd << ", ";
								sadd << "numDipoles = " << i->numPoints;
								needComma = true;
							}
							if (i->desc.size())
							{
								if (needComma) sadd << ", ";
								sadd << "description = '" << i->desc << "' ";
								needComma = true;
							}
							if (refId.size())
							{
								if (needComma) sadd << ", ";
								sadd << "flake_references = '" << refId << "' ";
								needComma = true;
							}
							sadd << " where hashlower = '" << i->hash().lower << "' ;";
						}
					}
				}

				sadd << "COMMIT;";
				std::string sres = sadd.str();
				h->execute(sres.c_str());


				return h;
			}
			*/
		}
	}
}
