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
			void psql_handle::handle_error(ConnStatusType status)
			{
				std::cerr << "psql library connection error " << status << std::endl;
				std::cerr << PQerrorMessage(connection.get()) << std::endl;
				RTthrow debug::xOtherError();
			}

			void psql_handle::handle_error(ExecStatusType status)
			{
				std::cerr << "psql library execution error " << status << std::endl;
				std::cerr << PQerrorMessage(connection.get()) << std::endl;
				RTthrow debug::xOtherError();
			}

			psql_handle::~psql_handle() {}

			psql_handle::psql_handle() : readable(true), writeable(true) {}

			void psql_handle::connect()
			{
				if (connection) return;
				const char* keywords[] = { "host", "dbname", "user", "password", "sslmode", "" };
				const char* values[] = { "plenus.met.fsu.edu", "rtmath", "rhoneyager", "", "require", "" };
				connection = boost::shared_ptr<PGconn>(PQconnectdbParams(keywords, values, 0), PQfinish);
				ConnStatusType errcode = PQstatus(connection.get());
				if (errcode != PGRES_COMMAND_OK) handle_error(errcode);
			}

			void psql_handle::disconnect()
			{
				connection.reset();
				connection = nullptr;
			}

			boost::shared_ptr<PGresult> psql_handle::execute(const char* command)
			{
				boost::shared_ptr<PGresult> res
					(PQexec(connection.get(), command), PQclear);
				ExecStatusType errcode = PQresultStatus(res.get());
				if (errcode != PGRES_COMMAND_OK && errcode != PGRES_TUPLES_OK) handle_error(errcode);
				return res;
			}


			void search(const rtmath::data::arm::arm_info_registry::arm_info_index &index,
				rtmath::data::arm::arm_info_registry::arm_info_index::collection res);
			void update(const rtmath::data::arm::arm_info_registry::arm_info_index::collection c,
				rtmath::data::arm::arm_info_registry::updateType t);
		}

	}
}


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

	doRegisterHook<arm_info, arm_query_registry, arm_info_registry>(reg_ai);
}
