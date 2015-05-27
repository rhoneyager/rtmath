/**
* \brief The debugging file, where all of the error-handling
* and versioning code resides.
**/

#include "Stdafx-core.h"
#include <iostream>
#include <sstream>
#include <thread>
#include <mutex>
#include <boost/version.hpp>
#include <boost/program_options.hpp>
#include <boost/exception/all.hpp>
#include <boost/log/sources/global_logger_storage.hpp>
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/hash.h>
#include <Ryan_Debug/logging.h>
#include <Ryan_Debug/error.h>
#include <Ryan_Debug/registry.h>
#include "../rtmath/config.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/registry.h"

#include "cmake-settings.h"

namespace {
	boost::program_options::options_description SHARED_PRIVATE *pcmdline = nullptr;
	boost::program_options::options_description SHARED_PRIVATE *pconfig = nullptr;
	boost::program_options::options_description SHARED_PRIVATE *phidden = nullptr;
	size_t sys_num_threads = 0;
	std::mutex m_sys_num_threads;



	BOOST_LOG_INLINE_GLOBAL_LOGGER_CTOR_ARGS(
		m_deb,
		boost::log::sources::severity_channel_logger_mt< >,
		(boost::log::keywords::severity = Ryan_Debug::log::error)(boost::log::keywords::channel = "debug"))
		;

}

namespace rtmath
{
	namespace debug
	{
		std::string SHARED_PRIVATE sConfigDefaultFile;

		size_t DLEXPORT_rtmath_core getConcurrentThreadsSupported()
		{
			std::lock_guard<std::mutex> lock(m_sys_num_threads);
			if (sys_num_threads) return sys_num_threads;
			sys_num_threads = static_cast<size_t> (std::thread::hardware_concurrency());
			if (!sys_num_threads) return 4;
			return sys_num_threads;
		}

		int rev(void)
		{
#if RTMATH_SVNREVISION
			return RTMATH_SVNREVISION;
#elif SUB_REV
			return SUB_REV;
#else
			return -1;
#endif
		};

		void debug_preamble(std::ostream &out)
		{
			out << "rtmath library" << std::endl;
			out << "Compiled on " << __DATE__ << " at " << __TIME__ << std::endl;
#ifdef SUB_REV
			out << "SVN Revision " << SUB_REV << std::endl;
			out << "SVN Revision Date: " << SUB_DATE << std::endl;
			//			out << "SVN Working Copy Range: " << SUB_WCRANGE << std::endl;
			out << "SVN Source: " << SUB_SOURCE << std::endl;
#else
			out << "SVN Repository Information Unknown" << std::endl;
#endif
		std::string buildtype(
				BUILDCONF
				); // defined in cmake config (addlib.cmake)
		out << "Build type: " << buildtype << std::endl;

#ifdef _OPENMP
			out << "OpenMP Supported" << std::endl;
#else
			out << "OpenMP Disabled" << std::endl;
#endif
#ifdef __amd64
			out << "64-bit build" << std::endl;
#endif
#ifdef _M_X64
			out << "64-bit build" << std::endl;
#endif
#ifdef __unix__
			out << "Unix / Linux Compile" << std::endl;
#endif
#ifdef __APPLE__
			out << "Mac Os X Compile" << std::endl;
#endif
#ifdef _WIN32
			out << "Windows Compile" << std::endl;
#endif
#ifdef __GNUC__
			out << "GNU Compiler Suite " << __GNUC__ << "." << __GNUC_MINOR__ << "." << __GNUC_PATCHLEVEL__ << std::endl;
#endif
#ifdef __MINGW32__
			out << "MinGW Compiler Suite " << __MINGW32_MAJOR_VERSION << "." << __MINGW32_MINOR_VERSION << std::endl;
#endif
#ifdef __SUNPRO_CC
			out << "Sun Studio Compiler " << __SUNPRO_CC << std::endl;
#endif
#ifdef __PATHCC__
			out << "EKOPath Compiler " << __PATHCC__ << "." << __PATHCC_MINOR__ << "." << __PATHCC_PATCHLEVEL__ << std::endl;
#endif
#ifdef __llvm__
			out << "LLVM Compiler Suite" << std::endl;
#endif
#ifdef __clang__
			out << " clang " << __clang_major__ << "." << __clang_minor__ << "." << __clang_patchlevel__ << std::endl;
#endif
#ifdef __INTEL_COMPILER
			out << "Intel Compiler Version " << __INTEL_COMPILER << std::endl;
			out << " Compiler build date: " << __INTEL_COMPILER_BUILD_DATE << std::endl;
#endif
#ifdef _MSC_FULL_VER
			out << "Microsoft Visual Studio Compiler Version " << _MSC_FULL_VER << std::endl;
#endif
			out << "Boost version " << BOOST_LIB_VERSION << std::endl;

			out << std::endl;
			out << std::endl;
		}
		
		void add_options(
			boost::program_options::options_description &cmdline,
			boost::program_options::options_description &config,
			boost::program_options::options_description &hidden)
		{
			namespace po = boost::program_options;
			using std::string;

			pcmdline = &cmdline;
			pconfig = &config;
			phidden = &hidden;

			cmdline.add_options()
				("version", "Print rtmath library version information and exit")
				;

			config.add_options()
				;
			
			hidden.add_options()
				("rtmath-config-file", po::value<std::string>(),
				"Specify the location of the rtmath configuration file. Overrides "
				"all other search locations. If it cannot be found, fall back to the "
				"next option.")
				("hash-dir", po::value<std::vector<string> >(), "Add a hash directory")
				("hash-dir-writable", po::value<bool>()->default_value(false), "Is the custom hash directory writable?")
				;

			rtmath::registry::add_options(cmdline, config, hidden);
		}

		
		void process_static_options(
			boost::program_options::variables_map &vm)
		{
			namespace po = boost::program_options;
			using std::string;
	
			auto& lg = m_deb::get();
			
			if (vm.count("rtmath-config-file"))
			{
				sConfigDefaultFile = vm["rtmath-config-file"].as<std::string>();
				BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Console override of rtmath-config-file: " << sConfigDefaultFile << "\n";
			}

			rtmath::config::loadRtconfRoot();

			if (vm.count("hash-dir"))
			{
				std::vector<string> hashDirs = vm["hash-dir"].as<std::vector<string> >();
				for (const auto &p : hashDirs)
				{
					std::shared_ptr<Ryan_Debug::hash::hashStore> h(new Ryan_Debug::hash::hashStore);
					h->writable = vm["hash-dir-writable"].as<bool>();
					h->base = boost::filesystem::path(p);
					BOOST_LOG_SEV(lg, Ryan_Debug::log::normal)
						<< "Console override of hash directory: " << p 
						<< ", writable: " << h->writable << ".\n";
					Ryan_Debug::hash::hashStore::addHashStore(h, 0);
				}
			}

			registry::process_static_options(vm);

			std::ostringstream preambles;
			preambles << "rtmath library information: \n";
			debug_preamble(preambles);
			preambles << "Ryan_Debug library information: \n";
			Ryan_Debug::printDebugInfo(preambles);

			std::string spreambles = preambles.str();

			if (vm.count("version"))
			{
				std::cerr << spreambles;
				exit(2);
			}


			BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << spreambles;
		}


	}
}
