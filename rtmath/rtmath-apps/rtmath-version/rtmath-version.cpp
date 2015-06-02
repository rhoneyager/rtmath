#include <iostream>
#include <fstream>
#include <boost/program_options.hpp>
#include <Ryan_Debug/debug.h>
#include "../../rtmath/rtmath/error/debug.h"
#include <Ryan_Debug/config.h>
#include "../../rtmath/rtmath/config.h"
#include <Ryan_Debug/logging.h>
#include <boost/log/sources/global_logger_storage.hpp>
//#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/formatter_parser.hpp>

BOOST_LOG_INLINE_GLOBAL_LOGGER_CTOR_ARGS(
	m_log,
	boost::log::sources::severity_channel_logger_mt< >,
	(boost::log::keywords::severity = Ryan_Debug::log::error)
	(boost::log::keywords::channel = "app"));

//namespace Ryan_Debug {
//	namespace log {
//		BOOST_LOG_ATTRIBUTE_KEYWORD(severity, "Severity", Ryan_Debug::log::severity_level);
//	}
//}


int main(int argc, char* argv[])
{
	using namespace std;
	using namespace rtmath;

	auto& lg = m_log::get();
	//boost::log::add_common_attributes();
	//boost::log::register_simple_formatter_factory< boost::log::trivial::severity_level, char >("Severity");
	//boost::log::register_simple_formatter_factory< ::Ryan_Debug::log::severity_level, char >("Severity");

	namespace po = boost::program_options;

	po::options_description desc("Allowed options"), cmdline("Command-line options"),
		config("Config options"), hidden("Hidden options"), oall("all options");
	//rtmath::ddscat::shapeFileStats::add_options(cmdline, config, hidden);
	Ryan_Debug::add_options(cmdline, config, hidden);
	rtmath::debug::add_options(cmdline, config, hidden);
	cmdline.add_options()
		("write-config", po::value<std::string>(), "Write the configuration to the specified location.")
		;

	rtmath::debug::debug_preamble();

	po::positional_options_description p;
	//p.add("output",2);

	desc.add(cmdline).add(config);
	oall.add(cmdline).add(config).add(hidden);

	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).
		options(oall).positional(p).run(), vm);
	po::notify(vm);

	Ryan_Debug::process_static_options(vm);
	rtmath::debug::process_static_options(vm);

	boost::log::register_simple_formatter_factory< Ryan_Debug::log::severity_level, char >("Severity");

	string sConfig;
	rtmath::config::getConfigDefaultFile(sConfig);
	if (sConfig.size())
		cerr << "Using rtmath configuration file: " << sConfig << endl << endl;
	else cerr << "rtmath configuration file not found." << endl << endl;
	
	Ryan_Debug::printDebugInfo();


	if (vm.count("write-config"))
	{
		std::string outfile = vm["write-config"].as<std::string>();
		auto conf = rtmath::config::getRtconfRoot();
		conf->writeFile(outfile);
		//BOOST_LOG_SEV(lg, normal) << "Console override of rtmath-config-file: " << sConfigDefaultFile << "\n";
	}

	BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_3) << "A debug_3";
	BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "A debug_2";
	BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_1) << "A debug_1";
	BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "A normal";
	BOOST_LOG_SEV(lg, Ryan_Debug::log::notification) << "A notification";
	BOOST_LOG_SEV(lg, Ryan_Debug::log::warning) << "A warning";
	BOOST_LOG_SEV(lg, Ryan_Debug::log::error) << "A error";
	BOOST_LOG_SEV(lg, Ryan_Debug::log::critical) << "A critical";

	/*
	BOOST_LOG_TRIVIAL(trace) << "A trace severity message";
	BOOST_LOG_TRIVIAL(debug) << "A debug severity message";
	BOOST_LOG_TRIVIAL(info) << "An informational severity message";
	BOOST_LOG_TRIVIAL(warning) << "A warning severity message";
	BOOST_LOG_TRIVIAL(error) << "An error severity message";
	BOOST_LOG_TRIVIAL(fatal) << "A fatal severity message";
	*/
	return 0;
}
