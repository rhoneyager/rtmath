/// \brief Provides psql database access
#define _SCL_SECURE_NO_WARNINGS

#include <algorithm>
#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/data/arm_info.h"
#include "../../rtmath/rtmath/data/arm_scanning_radar_sacr.h"
#include "../../rtmath/rtmath/plugin.h"

#include "plugin-psql.h"

void dllEntry();
rtmath_plugin_init(dllEntry);

namespace rtmath
{
	namespace plugins
	{
		namespace psql
		{
			void psql_handle::handle_error(ConnStatusType status) const
			{
				std::cerr << "psql library connection error " << status << std::endl;
				std::cerr << PQerrorMessage(connection.get()) << std::endl;
				RTthrow debug::xOtherError();
			}

			void psql_handle::handle_error(ExecStatusType status) const
			{
				std::cerr << "psql library execution error " << status << std::endl;
				std::cerr << PQerrorMessage(connection.get()) << std::endl;
				RTthrow debug::xOtherError();
			}

			void psql_handle::handle_error(const char* err) const
			{
				std::cerr << "psql library execution error: " << err << std::endl;
				RTthrow debug::xOtherError();
			}

			std::string psql_handle::escString(const std::string &in)
			{
				int error = 0;
				size_t outmxlen = (in.size() * 2) + 1;
				char *to = new char[outmxlen];
				size_t outlen = PQescapeStringConn(connection.get(), to, in.c_str(), in.size(), &error);
				std::string res(to);
				delete[] to;

				if (error) handle_error("Bad input string encoding.");
				
				return res;
			}

			psql_handle::~psql_handle() {}

			psql_handle::psql_handle(std::shared_ptr<registry::DB_options> o) : readable(true), writeable(true), 
				rtmath::registry::DBhandler(PLUGINID), o(o)
			{
				if (!o) this->o = registry::DB_options::generate();
			}

			void psql_handle::connect()
			{
				if (connection) return;
				using namespace std;
				vector<string> keywords, values;
				keywords.push_back("host"); keywords.push_back("dbname"); keywords.push_back("user");
				keywords.push_back("password"); keywords.push_back("sslmode"); keywords.push_back("");

				values.push_back(o->getVal<std::string>("hostname", "plenus.met.fsu.edu"));
				values.push_back(o->getVal<std::string>("dbname", "rtmath"));
				values.push_back(o->getVal<string>("user","rtmath_test"));
				values.push_back(o->getVal<string>("password","kjASDGK7nwk83g8gnKIu2g3neiY"));
				values.push_back(o->getVal<string>("sslmode","require"));
				values.push_back("");

				vector<const char*> ck, cv;
				for (const auto &s : keywords) ck.push_back(s.c_str());
				for (const auto &s : values) cv.push_back(s.c_str());

				//connection = std::shared_ptr<PGconn>(PQconnectdbParams(ck.data(), cv.data(), 0), PQfinish);
				connection = boost::shared_ptr<PGconn>(PQconnectdb(
					"host = plenus.met.fsu.edu dbname = rtmath user = rtmath_test sslmode = require password = kjASDGK7nwk83g8gnKIu2g3neiY"), PQfinish);
				ConnStatusType errcode = PQstatus(connection.get());
				if (errcode != CONNECTION_OK) handle_error(errcode);
			}

			void psql_handle::disconnect()
			{
				connection.reset();
				connection = nullptr;
			}

			boost::shared_ptr<PGresult> psql_handle::execute(const char* command)
			{
				connect();
				boost::shared_ptr<PGresult> res
					(PQexec(connection.get(), command), PQclear);
				ExecStatusType errcode = PQresultStatus(res.get());
				if (errcode != PGRES_COMMAND_OK && errcode != PGRES_TUPLES_OK) handle_error(errcode);
				return res;
			}

			void psql_handle::sendQuery(const char* command)
			{
				connect();
				int res = PQsendQuery(connection.get(), command);
				if (!res)
				{
					const char* err = PQerrorMessage(connection.get());
					handle_error(err);
				}
			}

			boost::shared_ptr<PGresult> psql_handle::getQueryResult()
			{
				connect();
				boost::shared_ptr<PGresult> res
					(PQgetResult(connection.get()), PQclear);
				ExecStatusType errcode = PQresultStatus(res.get());
				if (errcode != PGRES_COMMAND_OK && errcode != PGRES_TUPLES_OK) handle_error(errcode);
				return res;
			}


			std::shared_ptr<rtmath::registry::DBhandler>
				search(const rtmath::data::arm::arm_info_registry::arm_info_index &index,
				rtmath::data::arm::arm_info_registry::arm_info_index::collection res,
				std::shared_ptr<rtmath::registry::DBhandler>, std::shared_ptr<registry::DB_options>);
			std::shared_ptr<rtmath::registry::DBhandler>
				update(const rtmath::data::arm::arm_info_registry::arm_info_index::collection c,
				rtmath::data::arm::arm_info_registry::updateType t,
				std::shared_ptr<rtmath::registry::DBhandler>, std::shared_ptr<registry::DB_options>);
			bool matches(std::shared_ptr<rtmath::registry::DBhandler>, std::shared_ptr<registry::DB_options>);
		}

	}
}

/*
std::shared_ptr<hdf5_handle> h = registry::construct_handle
<registry::IOhandler, hdf5_handle>(
sh, PLUGINID, [&](){return std::shared_ptr<hdf5_handle>(
new hdf5_handle(filename.c_str(), iotype)); });
*/

void dllEntry()
{
	using namespace rtmath::registry;
	static const rtmath::registry::DLLpreamble id(
		"Plugin-psql",
		"Provides postgresql database access (used for indexing stuff)",
		PLUGINID);
	rtmath_registry_register_dll(id);

	using namespace rtmath::data::arm;

	arm_info_registry reg_ai;
	reg_ai.name = "psql-" PLUGINID;
	reg_ai.fInsertUpdate = rtmath::plugins::psql::update;
	reg_ai.fQuery = rtmath::plugins::psql::search;
	reg_ai.fMatches = rtmath::plugins::psql::matches;

	doRegisterHook<arm_info, arm_query_registry, arm_info_registry>(reg_ai);
}