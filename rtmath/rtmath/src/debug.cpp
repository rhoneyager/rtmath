/* debug.cpp
* The debugging file, where all of the error-handling
* and versioning code resides.
*/

#include "Stdafx-core.h"
#include <iostream>
#include <sstream>
#include <boost/version.hpp>
#include <boost/program_options.hpp>
#include <Ryan_Debug/debug.h>
#include "../rtmath/error/debug.h"
#include "../rtmath/error/debug_mem.h"

// This file just defines the subversion revision, created at a pre-build strp
#include "debug_subversion.h"
#ifdef WITH_CMAKE
#include "cmake-settings.h"
#endif

namespace {
	boost::program_options::options_description SHARED_PRIVATE *pcmdline = nullptr;
	boost::program_options::options_description SHARED_PRIVATE *pconfig = nullptr;
	boost::program_options::options_description SHARED_PRIVATE *phidden = nullptr;
}

namespace rtmath
{
	namespace debug
	{
		void dumpErrorLocation(std::ostream &out)
		{
			out << "File: " << memcheck::__file__ << std::endl;
			out << "Line: " << memcheck::__line__ << std::endl;
			out << "Caller: " << memcheck::__caller__ << std::endl;
			out << std::endl;
		}

		int rev(void)
		{
#ifndef SUB_REV
			return -1;
#else
			return SUB_REV;
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
#ifdef _DEBUG
			out << "Debug Version" << std::endl;
#else
			out << "Release Version" << std::endl;
#endif
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
				;

			config.add_options()
				;
			
			hidden.add_options()
				("help-verbose", "Print out all possible program options")
				;
		}

		void process_static_options(
			boost::program_options::variables_map &vm)
		{
			namespace po = boost::program_options;
			using std::string;
			
			if (vm.count("help-verbose"))
			{
				po::options_description oall("All Options");
				oall.add(*pcmdline).add(*pconfig).add(*phidden);

				std::cerr << oall << std::endl;
				exit(2);
			}

			if (vm.count("version"))
			{
				std::cerr << "rtmath library information: \n";
				debug_preamble(std::cerr);
				/// \todo Add serialization and tmatrix information
				std::cerr << "Ryan_Debug library information: \n";
				Ryan_Debug::printDebugInfo();
				exit(2);
			}
		}


	}
}
