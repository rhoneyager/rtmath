/**
* \brief Contains registry functions for extending functionality through DLLs.
*
* Contains both general and OS-specific functions.
**/

#include "Stdafx-core.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <boost/version.hpp>
#include <boost/program_options.hpp>
#include <Ryan_Debug/debug.h>
#include "../rtmath/error/debug.h"
#include "../rtmath/error/debug_mem.h"
#include "../rtmath/error/error.h"

/*
/// \todo Combine all these instances in the rtmath/debug namespace
namespace {
	boost::program_options::options_description SHARED_PRIVATE *pcmdline = nullptr;
	boost::program_options::options_description SHARED_PRIVATE *pconfig = nullptr;
	boost::program_options::options_description SHARED_PRIVATE *phidden = nullptr;
}
*/

namespace rtmath
{
	namespace registry
	{

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
				("dll-load", po::value<std::vector<std::string> >(),
				"Specify dlls to load. If passed a directory, it loads all dlls present (one-level)")
				("no-default-dll-locations", "Prevent non-command line dll locations from being read")
				;
		}

		void process_static_options(
			boost::program_options::variables_map &vm)
		{
			namespace po = boost::program_options;
			using std::string;

		}

	}
}
