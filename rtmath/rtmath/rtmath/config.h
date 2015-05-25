#pragma once
#include "defs.h"

#include <string>
#include <iostream>
#include <boost/shared_ptr.hpp>

namespace Ryan_Debug {
	namespace config {
		class configsegment;
	}
}
namespace rtmath {
	
	/** \brief describes the structure that should be contained in options files that
	 * involve the rtmath library.
	 * 
	 * This is to avoid having so create system-specific answer files or 
	 * having to reenter options on each execution. The actual configuration files
	 * will be based on the structure of Apache's httpd.conf.
	 *
	 * There are general options and specific containers. The file can reference other files
	 * as well. Global options are visible in the subcontainers, though these may be
	 * overridden for granularity and to allow for reusing files between multiple machines.
	 * The rtmath main program will parse the file and look for it in a set of pre-programmed
	 * locations upon execution.
	 *
	 * Once a main config file is read, the config options specify the next step. Daemons
	 * will prepare themselves for runs and will then wait for a connection. Further
	 * instructions will be transmitted in a structure similar to that of the standard
	 * config file. That necessary execute blurb, if present in a console application,
	 * will determine what the app does. If missing, an interactive app will just ask
	 * the user what to do.
	 **/
	namespace config {
		
		void DLEXPORT_rtmath_core getConfigDefaultFile(std::string &filename);
		boost::shared_ptr<::Ryan_Debug::config::configsegment> DLEXPORT_rtmath_core getRtconfRoot();
		/// Load the appropriate default rtmath configuration file (default may be overridden in command line, see registry.cpp)
		boost::shared_ptr<::Ryan_Debug::config::configsegment> DLEXPORT_rtmath_core loadRtconfRoot(const std::string &filename = "");
		void DLEXPORT_rtmath_core setRtconfRoot(boost::shared_ptr<::Ryan_Debug::config::configsegment> &root);

	}
}

