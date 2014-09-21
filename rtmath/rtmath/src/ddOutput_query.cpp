#include "Stdafx-ddscat.h"
#pragma warning( disable : 4996 ) // -D_SCL_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <tuple>
#include <boost/math/constants/constants.hpp>
#include <boost/lexical_cast.hpp>


//#include <boost/uuid/uuid.hpp>
//#include <boost/uuid/uuid_generators.hpp>
//#include <boost/uuid/uuid_io.hpp>

#include <Ryan_Debug/debug.h>
#include "../rtmath/macros.h"
#include "../rtmath/hash.h"
#include "../rtmath/ddscat/ddOutput.h"
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/ddscat/ddpar.h"
#include "../rtmath/splitSet.h"
#include "../rtmath/registry.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"

namespace rtmath {

	namespace ddscat {

		/*
		std::string ddOutput::genUUID() const
		{
			//using namespace boost::uuids;
			//random_generator gen;
			//uuid u = gen();
			//string suuid = boost::lexical_cast<string>(u);

			std::ostringstream out;
			out << shapeHash.lower << "-";
			if (runuuid.size()) {
				out << runuuid;
				std::string res = out.str();
				return res;
			} else return genName();
		}
		*/

		bool ddOutput_db_registry::ddOutput_db_comp::operator()(const std::shared_ptr<const ddOutput>& lhs, const std::shared_ptr<const ddOutput>& rhs) const
		{
			return *lhs < *rhs;
		}

		bool ddOutput_db_registry::ddOutput_db_comp::operator()(const boost::shared_ptr<const ddOutput>& lhs, const boost::shared_ptr<const ddOutput>& rhs) const
		{
			return *lhs < *rhs;
		}


		bool ddOutput::operator!=(const ddOutput &rhs) const { return !operator==(rhs); }
		bool ddOutput::operator==(const ddOutput &rhs) const {
#define check(x) if (x != rhs.x) return false;
			check(shapeHash);
			check(freq); check(temp); check(aeff); check(numOriData);
			return true;
#undef check
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

		std::pair<ddOutput_db_registry::ddOutput_index::collection, std::shared_ptr<rtmath::registry::DBhandler> >
			ddOutput_db_registry::ddOutput_index::doQuery(std::shared_ptr<rtmath::registry::DBhandler> p, std::shared_ptr<registry::DB_options> o) const
		{
			collection c(new std::set<boost::shared_ptr<const ddOutput>, ddOutput_db_comp >());
			std::shared_ptr<rtmath::registry::DBhandler> fp;

			auto hooks = ::rtmath::registry::usesDLLregistry<ddOutput_query_registry, ddOutput_db_registry >::getHooks();
			for (const auto &h : *(hooks.get()))
			{
				if (!h.fQuery) continue;
				if (!h.fMatches) continue;
				if (!h.fMatches(p, o)) continue;
				fp = h.fQuery(*this, c, p, o);

				return std::pair < ddOutput_db_registry::ddOutput_index::collection,
					std::shared_ptr<rtmath::registry::DBhandler> > (c, fp);
			}

			return std::pair<ddOutput_db_registry::ddOutput_index::collection,
				std::shared_ptr<rtmath::registry::DBhandler> >
				(c, nullptr);
		}


		bool ddOutput_db_registry::ddOutput_index::filter(const ddOutput* s) const
		{
			//std::cerr << "Filtering " << s->shapeHash.string() << std::endl;
			// hash filtering
			if (hashLowers.size() && !hashLowers.count(s->shapeHash.string()))
				return false;
			if (hashUppers.size() && !hashUppers.count(boost::lexical_cast<std::string>(s->shapeHash.upper)))
				return false;

			if (runids.size() && !runids.count(s->runhash().string()))
				return false;

			// flake types
			if (flakeTypes.size() && s->tags.count("flake_classification"))
				if (!flakeTypes.count(s->shape->tags.at("flake_classification")))
					return false;

			// polarizations
			if (pol.size())
				if (!pol.count(s->parfile->namePolState()))
					return false;

			ddscat::rotations rots;
			s->parfile->getRots(rots);

			if (betaRanges.ranges.size())
				if (!betaRanges.inRange(rots.bN())) return false;
			if (thetaRanges.ranges.size())
				if (!thetaRanges.inRange(rots.tN())) return false;
			if (phiRanges.ranges.size())
				if (!phiRanges.inRange(rots.pN())) return false;


			if (aeffRanges.ranges.size())
				if (!aeffRanges.inRange(static_cast<float>(s->aeff))) return false;

			if (tempRanges.ranges.size())
				if (!tempRanges.isNear(static_cast<float>(s->temp), 1, 0)) return false; // within 1 K

			if (freqRanges.ranges.size())
				if (!freqRanges.isNear(static_cast<float>(s->freq), 0, 0.01f)) return false; // within 1%


			// Run already has dipole spacings and number of dipoles. No need to load the shape.
			// Number of dipoles
			if (dipoleNumbers.ranges.size())
				if (!dipoleNumbers.inRange(s->s.num_dipoles)) return false;

			// Dipole spacings
			if (dipoleSpacings.ranges.size())
			{
				double V_um = pow(s->aeff, 3.) * 4. * boost::math::constants::pi<double>() / 3.;
				double V_di = (double)s->s.num_dipoles;
				double ds = pow(V_um / V_di, 1. / 3.);
				if (!dipoleSpacings.inRange((float)ds)) return false;
			}

			
			return true;
		}

		std::pair<ddOutput_db_registry::ddOutput_index::collection,
			std::shared_ptr<rtmath::registry::DBhandler> >
			ddOutput_db_registry::ddOutput_index::doQuery(
			collection srcs, bool doUnion, bool doDb,
			std::shared_ptr<rtmath::registry::DBhandler> p,
			std::shared_ptr<registry::DB_options> o) const
		{
			collection toMergeC(new std::set<boost::shared_ptr<const ddOutput>, ddOutput_db_comp >());
			//collection toMergeS(new std::set<boost::shared_ptr<ddOutput>, ddOutput_db_comp >());
			collection res(new std::set<boost::shared_ptr<const ddOutput>, ddOutput_db_comp >());
			std::shared_ptr<rtmath::registry::DBhandler> fp;

			std::map<HASH_t, boost::shared_ptr<const ddOutput> > db_hashes; // database results, as a map

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
					db_hashes[r->runhash()] = r;
			}



			// Also perform the same filtering on srcs, putting results in res
			for (auto &s : *(srcs))
			{
				if (!filter(s.get())) continue;

				res->insert(s); // Passed filtering
				// Merge the results of the provided object with any query results
				if (db_hashes.count(s->runhash()))
				{
					auto d = db_hashes.at(s->runhash());

					db_hashes.erase(s->runhash()); // Remove from consideration (already matched)
					db_hashes[s->runhash()] = s;
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

