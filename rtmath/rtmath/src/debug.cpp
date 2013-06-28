/* debug.cpp
* The debugging file, where all of the error-handling
* and versioning code resides.
*/

#include "../rtmath/Stdafx.h"
#include <iostream>
#include <sstream>
#include <boost/version.hpp>
#include "../rtmath/error/debug.h"
#include "../rtmath/error/debug_mem.h"

// This file just defines the subversion revision, created at a pre-build strp
#include "debug_subversion.h"
#ifdef WITH_CMAKE
#include "cmake-settings.h"
#endif

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
		};

	} // end debug
} // end rtmath
