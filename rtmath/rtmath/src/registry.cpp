/**
* \brief Contains registry functions for extending functionality through DLLs.
*
* Contains both general and OS-specific functions.
**/

#include "Stdafx-core.h"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/version.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>
#include <boost/log/sources/global_logger_storage.hpp>
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/fs.h>
#include "../rtmath/config.h"
#include "../rtmath/splitSet.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/registry.h"

using namespace rtmath::debug;
namespace rtmath
{
	namespace registry
	{
		/// Recursive and single-level DLL loading paths
		std::set<boost::filesystem::path> searchPathsRecursive, searchPathsOne;

		BOOST_LOG_INLINE_GLOBAL_LOGGER_CTOR_ARGS(
			m_reg,
			blog::sources::severity_channel_logger_mt< >,
			(blog::keywords::severity = error)(blog::keywords::channel = "registry"));

		void emit_registry_log(const std::string &m, ::rtmath::debug::severity_level sev)
		{
			auto& lg = rtmath::registry::m_reg::get();
			BOOST_LOG_SEV(lg, sev) << m;
		}

		void add_options(
			boost::program_options::options_description &cmdline,
			boost::program_options::options_description &config,
			boost::program_options::options_description &hidden)
		{
			namespace po = boost::program_options;
			using std::string;

			cmdline.add_options()
				;

			config.add_options()
				;

			hidden.add_options()
				;
		}

		/** \todo Needs to be restructured to take advantage of Ryan_Debug code.
		* Ryan_Debug provides dll search and load functions. So, this should consult the 
		* rtmath.conf file and the core system to find subdirectories to search for DLLs.
		* Then, it should load these dlls with a custom validator that ensures that the 
		* correct rtmath version is being bound.
		**/
		void process_static_options(
			boost::program_options::variables_map &vm)
		{
			namespace po = boost::program_options;
			using std::string;

			auto& lg = m_reg::get();
			BOOST_LOG_SEV(lg, normal) << "Initializing registry system\n";

			//if (vm.count("dll-no-default-locations"))
			//	autoLoadDLLs = false;

			if (vm.count("dll-load-onelevel"))
			{
				std::vector<std::string> sPaths = vm["dll-load-onelevel"].as<std::vector<std::string> >();
				BOOST_LOG_SEV(lg, notification) << "Loading custom dll paths (1)\n";
				for (const auto s : sPaths)
				{
					searchPathsOne.emplace(s);
					BOOST_LOG_SEV(lg, notification) << "Loading custom dll path (1): " << s << "\n";
				}
			}

			if (vm.count("dll-load-recursive"))
			{
				std::vector<std::string> sPaths = vm["dll-load-recursive"].as<std::vector<std::string> >();
				BOOST_LOG_SEV(lg, notification) << "Loading custom dll paths (1)" << "\n";
				for (const auto s : sPaths)
				{
					searchPathsRecursive.emplace(s);
					BOOST_LOG_SEV(lg, notification) << "Loading custom dll path (r): " << s << "\n";
				}
			}

			constructSearchPaths(false, true, true);

			if (vm.count("print-dll-search-paths"))
				printDLLsearchPaths(std::cerr);

			std::set<boost::filesystem::path> rPaths, oPaths;
			findPath(rPaths, boost::filesystem::path("default"), searchPathsRecursive, true);
			findPath(oPaths, boost::filesystem::path("default"), searchPathsOne, false);
			std::vector<std::string> toLoadDlls;
			// If a 'default' folder exists in the default search path, then use it for dlls.
			// If not, then use the base plugins directory.
			// Any library version / name detecting logic is in loadDLL (called by loadDLLs).

			if (rPaths.size() || oPaths.size())
			{
				searchDLLs(toLoadDlls, rPaths, true);
				searchDLLs(toLoadDlls, oPaths, false);
			}
			else { searchDLLs(toLoadDlls); }

			loadDLLs(toLoadDlls);
			

			if (vm.count("print-dll-loaded"))
				printDLLs();
		}

	}
}
