/* debug.cpp
 * The debugging file, where all of the error-handling
 * and versioning code resides.
 */

#include "stdafx.h"
#include "debug.h"
#include "error.h"
#include <map>
#include <iostream>
#include <string>
#include <sstream>

namespace rtmath
{
	namespace debug
	{
		std::map<keymap*, int> UNIQUE_KEYS;

		int rev(void)
		{
#ifndef REV
			return -1;
#else
			return REV;
#endif
		};

		void debug_preamble(void)
		{
			std::cerr << "rtmath library" << std::endl;
			std::cerr << "Compiled on " << __DATE__ << " at " << __TIME__ << std::endl;
#ifdef REV
			std::cerr << "SVN Revision " << REV << std::endl;
#else
			std::cerr << "SVN Revision Unknown" << std::endl;
#endif
#ifdef _DEBUG
			std::cerr << "Debug Version" << std::endl;
#else
			std::cerr << "Release Version" << std::endl;
#endif
#ifdef __amd64
			std::cerr << "64-bit build" << std::endl;
#endif
#ifdef _M_X64
			std::cerr << "64-bit build" << std::endl;
#endif
#ifdef __unix__
			std::cerr << "Unix / Linux Compile" << std::endl;
#endif
#ifdef __APPLE__
			std::cerr << "Mac Os X Compile" << std::endl;
#endif
#ifdef _WIN32
			std::cerr << "Windows Compile" << std::endl;
#endif
#ifdef __GNUC__
			std::cerr << "GNU Compiler Suite " << __GNUC__ << "." << __GNUC_MINOR__ << "." << __GNUC_PATCHLEVEL__ << std::endl;
#endif
#ifdef __MINGW32__
			std::cerr << "MinGW Compiler Suite " << __MINGW32_MAJOR_VERSION << "." << __MINGW32_MINOR_VERSION << std::endl;
#endif
#ifdef __SUNPRO_CC
			std::cerr << "Sun Studio Compiler " << __SUNPRO_CC << std::endl;
#endif
#ifdef __llvm__
			std::cerr << "LLVM Compiler Suite" << std::endl;
#endif
#ifdef _MSC_FULL_VER
			std::cerr << "Microsoft Visual Studio Compiler Version " << _MSC_FULL_VER << std::endl;
#endif
			std::cerr << std::endl;
		};

		unsigned int getuniqueobj(const char* file, int line, const char* func)
		{
			keymap* temp = new keymap(file,line,func);
    
			if (UNIQUE_KEYS.count(temp) == 0)
			{
				UNIQUE_KEYS[temp] = 0;
			}
			return UNIQUE_KEYS[temp]++;
		}

		void listuniqueobj(void)
		{
			// This is a debugging function that will output to cerr the keymaps, counts and identities
			using namespace std;
			cerr << "\nUnique object keymap listing:\n";
			cerr << UNIQUE_KEYS.size() << " objects\n";
			cerr << "File - Function - Line - Count\n";
			for (map<keymap*,int>::iterator it = UNIQUE_KEYS.begin(); it != UNIQUE_KEYS.end(); it++)
			{
				cerr << it->first->file << " - " << it->first->function << " - " << it->first->line << " - " << it->second << endl;
        
			}
			cerr << endl;
		}

		std::string diemsg(const char* file, int line, const char* sig)
		{
			using namespace std;
			std::ostringstream message(ostringstream::out);
			message << "---FATAL ERROR--- \nExecution stopped in " << sig << endl
				<< " in " << file << endl
				<< " at line " << line << endl << endl;
			return message.str();
		}

	}; // end debug
}; // end rtmath