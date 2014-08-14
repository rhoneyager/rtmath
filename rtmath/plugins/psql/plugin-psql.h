#pragma once
#pragma warning( disable : 4251 ) // warning C4251: dll-interface

#include <memory>
#include <string>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"

#include <libpq-fe.h>

#define PLUGINID "8B0AA4D8-AF0E-4EBD-8452-974AD0342325"

namespace rtmath {
	namespace ddscat {
		class ddOutput;
		class ddOutputSingle;
		class ddScattMatrix;
		namespace shapefile { class shapefile; }
		namespace stats { class shapeFileStats; }
	}
	namespace plugins {
		/** \brief Namespace for the psql database handler.
		*
		* This plugin will provide an interface to the postgres database.
		* It will make only one connection to the database (per thread).
		* Each connection will remain persistent until either manually 
		* closed or until all handles have been invalidated?
		**/
		namespace psql {
			
			struct psql_handle : public rtmath::registry::DBhandler
			{
				psql_handle(std::shared_ptr<registry::DB_options>);
				virtual ~psql_handle();

				std::shared_ptr<PGresult> execute(const char* command);
				void sendQuery(const char* command);
				std::shared_ptr<PGresult> getQueryResult();
			private:
				void connect();
				void disconnect();
				bool readable;
				bool writeable;
				void handle_error(ConnStatusType status) const;
				void handle_error(ExecStatusType status) const;
				void handle_error(const char* err) const;
				//std::shared_ptr<psqlFile> file;
				std::shared_ptr<PGconn> connection;
				std::shared_ptr<registry::DB_options> o;
			};


		}
	}
}

