/**
* \brief The debugging file, where all of the error-handling
* and versioning code resides.
**/
#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <sstream>
#include <thread>
#include <mutex>
#include <boost/version.hpp>
#include <boost/program_options.hpp>
#include <boost/exception/all.hpp>
//#include <boost/core/null_deleter.hpp> // This is the correct one
#include "../Ryan_Debug/deleter.h" // Sick of so many different locations for this.
//#include <boost/utility/empty_deleter.hpp>
//#include <boost/serialization/shared_ptr.hpp> // provides null_deleter for older boost versions
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
#include <boost/log/attributes/clock.hpp>
#include <boost/log/expressions/formatters/date_time.hpp>
#include <boost/log/support/date_time.hpp>
#include "../Ryan_Debug/debug.h"
#include "../Ryan_Debug/config.h"
#include "../Ryan_Debug/debug.h"
#include "../Ryan_Debug/error.h"
#include "../Ryan_Debug/logging.h"
#include "../Ryan_Debug/registry.h"
#include "../Ryan_Debug/hash.h"
#include "internal.h"

// This file just defines the subversion revision, created at a pre-build strp
#include "debug_subversion.h"
#include "cmake-settings.h"

namespace blog = boost::log;
namespace {
	boost::program_options::options_description *pcmdline = nullptr;
	boost::program_options::options_description *pconfig = nullptr;
	boost::program_options::options_description *phidden = nullptr;
	size_t sys_num_threads = 0;
	std::mutex m_sys_num_threads;



	BOOST_LOG_INLINE_GLOBAL_LOGGER_CTOR_ARGS(
		m_deb,
		blog::sources::severity_channel_logger_mt< >,
		(blog::keywords::severity = Ryan_Debug::log::error)(blog::keywords::channel = "debug"))
		;

	// Both logging systems go here
	typedef boost::log::sinks::synchronous_sink< boost::log::sinks::text_ostream_backend > text_sink;
	static boost::shared_ptr< text_sink > sink_init, sink;
}

namespace Ryan_Debug
{
	namespace debug
	{
		std::string sConfigDefaultFile;
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

			/// \todo Add option for default Ryan_Debug.conf location

			cmdline.add_options()
				("version", "Print Ryan_Debug library version information and exit")
				("close-on-finish", po::value<bool>(), "Should the app automatically close on termination?")
				;

			config.add_options()
				;
			
			hidden.add_options()
				("help-verbose", "Print out all possible program options")
				("help-all", "Print out all possible program options")
				("help-full", "Print out all possible program options")
				//("log-init", "Log initial startup")
				("log-level-all", po::value<int>()->default_value((int)::Ryan_Debug::log::warning), "Threshold for console logging")
				("Ryan_Debug-config-file", po::value<std::string>(),
				"Specify the location of the Ryan_Debug configuration file. Overrides "
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
			return level >= Ryan_Debug::debug::warning; // || tag == "IMPORTANT_MESSAGE";
		}*/

		void setupLoggingInitial(int logthresholdlevel = ::Ryan_Debug::log::warning)
		{
			static bool setup = false;
			if (setup) return;
			setup = true;

			::boost::log::core::get()->add_global_attribute(
				"TimeStamp",
				::boost::log::attributes::local_clock());

			sink_init = boost::make_shared< text_sink >();

			// We have to provide an empty deleter to avoid destroying the global stream object
			// boost::serialization::null_deleter(), boost::empty_deleter(), boost::null_deleter() use varies with boost version...
			boost::shared_ptr< std::ostream > stream(&std::cerr, boost::null_deleter());
			sink_init->locked_backend()->add_stream(stream);
			//if (!logall) {
			sink_init->set_filter(Ryan_Debug::log::severity >= logthresholdlevel //debug_3 //warning
				//boost::log::expressions::attr < int >
				//("Severity").or_default(Ryan_Debug::debug::normal)
				); // Ryan_Debug::debug::warning);
			//}
			sink_init->set_formatter( boost::log::expressions::stream 
				<< boost::log::expressions::format_date_time
					< boost::posix_time::ptime >("TimeStamp", "%Y-%m-%d %H:%M:%S")
					<< ": <" << (Ryan_Debug::log::severity)
					<< "> [" << Ryan_Debug::log::channel << "] {init} "
				<< boost::log::expressions::smessage

				);

			
			boost::shared_ptr< boost::log::core > core = boost::log::core::get();
			core->add_sink(sink_init);
			//boost::log::core::get()->set_filter(Ryan_Debug::debug::severity_level >= Ryan_Debug::debug::warning);


#ifdef _WIN32
			// Complete sink type
			typedef boost::log::sinks::synchronous_sink< boost::log::sinks::debug_output_backend > d_sink_t;
			// Create the debugger sink. The backend requires synchronization in the frontend.
			boost::shared_ptr< d_sink_t > d_sink(new d_sink_t());

			// Set the special filter to the frontend
			// in order to skip the sink when no debugger is available
			d_sink->set_filter(boost::log::expressions::is_debugger_present());
			d_sink->set_formatter(boost::log::expressions::stream
				<< boost::log::expressions::format_date_time
				< boost::posix_time::ptime >("TimeStamp", "%Y-%m-%d %H:%M:%S")
				<< ": <" << (Ryan_Debug::log::severity)
				<< "> [" << Ryan_Debug::log::channel << "] {debugger} "
				<< boost::log::expressions::smessage << std::endl);

			core->add_sink(d_sink);
#endif
			
			auto& lg = m_deb::get();
			BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Initial logging started.";
			core->flush();
		}


		void setupLogging(boost::program_options::variables_map &vm, int logthresholdlevel = ::Ryan_Debug::log::warning)
		{
			static bool setup = false;
			if (setup) return;
			// First, use any console overrides
			/*

			// Using the parameters from the Ryan_Debug config (and the console options),
			// create the logging sinks.

			auto conf = Ryan_Debug::config::loadRtconfRoot();
			auto cdd = conf->getChild("General");
			if (!cdd) return;
			auto clogs = cdd->getChild("Logging");

			

			// Iterate over all hash store entries
			std::multiset<boost::shared_ptr<Ryan_Debug::config::configsegment> > children;
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
					RDthrow debug::xBadInput("Parsing error in log configuration");
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
							else RDthrow debug::xBadInput("Parsing error in log configuration");

							auto checkSeverityThreshold = [&](bool, debug::severity_level minsl) -> bool
							{

							};
						}
					}
				}
			}
			*/
			// Construct the console sink
			sink = boost::make_shared< text_sink >();

			boost::shared_ptr< boost::log::core > core = boost::log::core::get();
			//core->remove_all_sinks(); // Remove initial sink
			// We have to provide an empty deleter to avoid destroying the global stream object
			// boost::serialization::null_deleter(), boost::empty_deleter(), boost::null_deleter() use varies with boost version...
			boost::shared_ptr< std::ostream > stream(&std::clog, boost::null_deleter());
			sink->locked_backend()->add_stream(stream);
			sink->set_filter(Ryan_Debug::log::severity >= logthresholdlevel //warning
				//boost::log::expressions::attr < int >
				//("Severity").or_default(Ryan_Debug::debug::normal)
				); // Ryan_Debug::debug::warning);
			sink->set_formatter( boost::log::expressions::stream 
				<< boost::log::expressions::format_date_time
					< boost::posix_time::ptime >("TimeStamp", "%Y-%m-%d %H:%M:%S")
					<< ": <" << Ryan_Debug::log::severity
					<< "> [" << Ryan_Debug::log::channel << "] "
				<< boost::log::expressions::smessage
				);

			
			core->add_sink(sink);
			core->flush();
			core->remove_sink(sink_init);


			//boost::log::core::get()->set_filter(Ryan_Debug::debug::severity_level >= Ryan_Debug::debug::warning);



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
	
			// Bring up a basic logging system for critical first-load library tasks,
			// like finding a configuration file.	

			int sevlev = (int) ::Ryan_Debug::log::warning; // default really set in add_static_options
			sevlev = vm["log-level-all"].as<int>();

			setupLoggingInitial(sevlev);

			if (vm.count("help-verbose") || vm.count("help-all") || vm.count("help-full"))
			{
				po::options_description oall("All Options");
				oall.add(*pcmdline).add(*pconfig).add(*phidden);

				std::cerr << oall << std::endl;
				exit(2);
			}
			
			auto& lg = m_deb::get();
			
			if (vm.count("Ryan_Debug-config-file"))
			{
				debug::sConfigDefaultFile = vm["Ryan_Debug-config-file"].as<std::string>();
				BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Console override of Ryan_Debug-config-file: " 
					<< debug::sConfigDefaultFile ;
			}

			//if (vm.count("Ryan_Debug-conf"))
			//{
			//	BOOST_LOG_SEV(lg, notification) << "Loading custom Ryan_Debug.conf from " << vm["Ryan_Debug-conf"].as<string>();
			//	Ryan_Debug::config::loadRtconfRoot(vm["Ryan_Debug-conf"].as<string>());
			//} else { Ryan_Debug::config::loadRtconfRoot(); }
			Ryan_Debug::config::loadRtconfRoot();

			//atexit(appExit);
			//Switch to more complete logging system.
			if (vm.count("close-on-finish")) {
				bool val = !(vm["close-on-finish"].as<bool>());
				Ryan_Debug::waitOnExit(val);
				BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Console override of waiting on exit: " << val ;
			}

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
						<< ", writable: " << h->writable << ".";
					Ryan_Debug::hash::hashStore::addHashStore(h, 0);
				}
			}

			BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Switching to primary logging system";
			setupLogging(vm, sevlev);
			BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Primary logging system started.";
			boost::shared_ptr< boost::log::core > core = boost::log::core::get();
			core->flush();


			registry::process_static_options(vm);

			std::ostringstream preambles;
			preambles << "Ryan_Debug library information: \n";
			Ryan_Debug::versioning::debug_preamble(preambles);

			std::string spreambles = preambles.str();

			if (vm.count("version"))
			{
				std::cerr << spreambles;
				exit(2);
			}


			BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << spreambles;
		}


	//}
}
