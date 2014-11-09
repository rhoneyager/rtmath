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
#include <boost/core/null_deleter.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions/predicates/is_debugger_present.hpp>
#include <boost/log/attributes.hpp>
#include <boost/log/expressions/keyword.hpp>
#include <boost/log/expressions/attr_fwd.hpp>
#include <boost/log/expressions/attr.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sources/channel_feature.hpp>
#include <boost/log/sources/channel_logger.hpp>
#include <boost/log/sources/severity_feature.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/severity_channel_logger.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/sinks/debug_output_backend.hpp>
#include <boost/log/sources/global_logger_storage.hpp>
#include <Ryan_Debug/debug.h>
#include "../rtmath/config.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"
//#include "../rtmath/error/debug_mem.h"
#include "../rtmath/registry.h"
#include "../rtmath/hash.h"

// This file just defines the subversion revision, created at a pre-build strp
#include "debug_subversion.h"
#ifdef WITH_CMAKE
#include "cmake-settings.h"
#endif

namespace blog = boost::log;
namespace {
	boost::program_options::options_description SHARED_PRIVATE *pcmdline = nullptr;
	boost::program_options::options_description SHARED_PRIVATE *pconfig = nullptr;
	boost::program_options::options_description SHARED_PRIVATE *phidden = nullptr;
	size_t sys_num_threads = 0;
	std::mutex m_sys_num_threads;



	BOOST_LOG_INLINE_GLOBAL_LOGGER_CTOR_ARGS(
		m_deb,
		blog::sources::severity_channel_logger_mt< >,
		(blog::keywords::severity = rtmath::debug::error)(blog::keywords::channel = "debug"))
}

namespace rtmath
{
	namespace debug
	{
		std::string SHARED_PRIVATE sConfigDefaultFile;

		void dumpErrorLocation(std::ostream &out)
		{
			out << "File: " << memcheck::__file__ << std::endl;
			out << "Line: " << memcheck::__line__ << std::endl;
			out << "Caller: " << memcheck::__caller__ << std::endl;
			out << std::endl;
		}

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

		boost::filesystem::path expandSymlink(const boost::filesystem::path &p)
		{
			using namespace boost::filesystem;
			// Set a max depth to avoid infinite loops
			const size_t maxDepth = 10;
			size_t d=0;
			path pf = p;
			while(is_symlink(pf) && d<maxDepth)
			{
				pf = boost::filesystem::absolute(read_symlink(pf), p.parent_path());
				d++;
			}
			return pf;
		}

		void expandFolder(const std::string &s, 
			std::vector<boost::filesystem::path> &out, bool recurse)
		{
			using namespace boost::filesystem;
			path p(s);
			expandFolder(p, out, recurse);
		}
		void expandFolder(const boost::filesystem::path &p, 
			std::vector<boost::filesystem::path> &dest, bool recurse)
		{
			using namespace boost::filesystem;
			if (is_directory(p))
			{
				if (!recurse)
					copy(directory_iterator(p), 
					directory_iterator(), back_inserter(dest));
				else
					copy(recursive_directory_iterator(p,symlink_option::recurse), 
					recursive_directory_iterator(), back_inserter(dest));
			}
			else dest.push_back(p);
		}
		void expandFolders(const std::vector<boost::filesystem::path> &src, 
			std::vector<boost::filesystem::path> &dest, bool recurse)
		{
			for (auto s : src)
				expandFolder(s, dest, recurse);
		}
		void expandFolders(const std::vector<std::string> &src, 
			std::vector<boost::filesystem::path> &dest, bool recurse)
		{
			for (auto s : src)
				expandFolder(s, dest, recurse);
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

			/// \todo Add option for default rtmath.conf location

			cmdline.add_options()
				("version", "Print rtmath library version information and exit")
				("close-on-finish", po::value<bool>(), "Should the app automatically close on termination?")
				;

			config.add_options()
				;
			
			hidden.add_options()
				("help-verbose", "Print out all possible program options")
				("help-all", "Print out all possible program options")
				("help-full", "Print out all possible program options")
				("rtmath-config-file", po::value<std::string>(),
				"Specify the location of the rtmath configuration file. Overrides "
				"all other search locations. If it cannot be found, fall back to the "
				"next option.")
				("hash-dir", po::value<std::vector<string> >(), "Add a hash directory")
				("hash-dir-writable", po::value<bool>()->default_value(false), "Is the custom hash directory writable?")
				;

			registry::add_options(cmdline, config, hidden);
		}

		/*bool stdlogFilter(boost::log::value_ref< severity_level, boost::log::tag::severity > const& level,
			boost::log::value_ref< std::string, boost::log::sources::tag::tag_attr > const& tag)
		{
			return level >= rtmath::debug::warning; // || tag == "IMPORTANT_MESSAGE";
		}*/



		void setupLogging(boost::program_options::variables_map &vm)
		{
			static bool setup = false;
			if (setup) return;

			// First, use any console overrides
			/*

			// Using the parameters from the rtmath config (and the console options),
			// create the logging sinks.

			auto conf = rtmath::config::loadRtconfRoot();
			auto cdd = conf->getChild("General");
			if (!cdd) return;
			auto clogs = cdd->getChild("Logging");

			

			// Iterate over all hash store entries
			std::multiset<boost::shared_ptr<rtmath::config::configsegment> > children;
			clogs->listChildren(children);
			for (const auto &c : children)
			{
				if (c->name() != "Sink") continue;

				bool enabled = true;
				if (c->hasVal("enabled"))
					c->getVal<bool>("enabled", enabled);
				if (!enabled) continue;

				std::string id;
				if (c->hasVal("id"))
					c->getVal<std::string>("id", id);
				else {
					RTthrow debug::xBadInput("Parsing error in log configuration");
				}

				// Make the filters
				auto subkeys = c->listChildren();
				for (const auto &i : subkeys)
				{
					if (i->name() == "filter")
					{
						// Construct a new filter, matching all of the traits
						if (c->hasVal("SeverityThreshold"))
						{
							debug::severity_level sl;
							std::string ssl;
							c->getVal<std::string>("SeverityThreshold", ssl);
							if (ssl == "normal") sl = debug::normal;
							else if (ssl == "notification") sl = debug::notification;
							else if (ssl == "warning") sl = debug::warning;
							else if (ssl == "error") sl = debug::error;
							else if (ssl == "critical") sl = debug::critical;
							else RTthrow debug::xBadInput("Parsing error in log configuration");

							auto checkSeverityThreshold = [&](bool, debug::severity_level minsl) -> bool
							{

							};
						}
					}
				}
			}
			*/
			// Construct the console sink
			typedef boost::log::sinks::synchronous_sink< boost::log::sinks::text_ostream_backend > text_sink;
			static boost::shared_ptr< text_sink > sink = boost::make_shared< text_sink >();

			// We have to provide an empty deleter to avoid destroying the global stream object
			boost::shared_ptr< std::ostream > stream(&std::clog, boost::null_deleter());
			sink->locked_backend()->add_stream(stream);
			sink->set_filter( severity >= warning
				//boost::log::expressions::attr < int >
				//("Severity").or_default(rtmath::debug::normal)
				); // rtmath::debug::warning);

			
			boost::shared_ptr< boost::log::core > core = boost::log::core::get();
			core->add_sink(sink);
			//boost::log::core::get()->set_filter(rtmath::debug::severity_level >= rtmath::debug::warning);





			// Complete sink type
			typedef boost::log::sinks::synchronous_sink< boost::log::sinks::debug_output_backend > d_sink_t;
			// Create the debugger sink. The backend requires synchronization in the frontend.
			boost::shared_ptr< d_sink_t > d_sink(new d_sink_t());

			// Set the special filter to the frontend
			// in order to skip the sink when no debugger is available
			d_sink->set_filter(boost::log::expressions::is_debugger_present());
			core->add_sink(d_sink);

			setup = true;
		}

		/*void appExit()
		{
			boost::shared_ptr< boost::log::core > core = boost::log::core::get();
			core->remove_all_sinks();
		}*/

		void process_static_options(
			boost::program_options::variables_map &vm)
		{
			namespace po = boost::program_options;
			using std::string;
			
			if (vm.count("help-verbose") || vm.count("help-all") || vm.count("help-full"))
			{
				po::options_description oall("All Options");
				oall.add(*pcmdline).add(*pconfig).add(*phidden);

				std::cerr << oall << std::endl;
				exit(2);
			}
			
			auto& lg = m_deb::get();
			
			if (vm.count("rtmath-config-file"))
			{
				sConfigDefaultFile = vm["rtmath-config-file"].as<std::string>();
				BOOST_LOG_SEV(lg, normal) << "Console override of rtmath-config-file: " << sConfigDefaultFile << "\n";
			}

			//atexit(appExit);
			setupLogging(vm);

			if (vm.count("close-on-finish")) {
				bool val = !(vm["close-on-finish"].as<bool>());
				Ryan_Debug::waitOnExit(val);
				BOOST_LOG_SEV(lg, normal) << "Console override of waiting on exit: " << val << "\n";
			}

			if (vm.count("hash-dir"))
			{
				std::vector<string> hashDirs = vm["hash-dir"].as<std::vector<string> >();
				for (const auto &p : hashDirs)
				{
					std::shared_ptr<hashStore> h(new hashStore);
					h->writable = vm["hash-dir-writable"].as<bool>();
					h->base = boost::filesystem::path(p);
					hashStore::addHashStore(h, 0);

					BOOST_LOG_SEV(lg, normal) << "Console override of hash directory: " << p << ", writable: " << h->writable << ".\n";
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


			BOOST_LOG_SEV(lg, normal) << spreambles;
		}


	}
}
