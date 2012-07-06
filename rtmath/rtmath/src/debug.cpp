/* debug.cpp
* The debugging file, where all of the error-handling
* and versioning code resides.
*/

#include "../rtmath/Stdafx.h"
#include <boost/version.hpp>
#include "../rtmath/error/debug.h"
#include "../rtmath/error/debug_mem.h"

// This file just defines the subversion revision, created at a pre-build strp
#include "debug_subversion.h"

namespace rtmath
{
	namespace debug
	{
		std::map<keymap, int> UNIQUE_KEYS;
		unsigned int uidtracker::_uidcount = 0;


		uidtracker::uidtracker()
		{
			_uid = _uidcount;
			_uidcount++;
			std::cerr << "Debug: new uid " << _uid << std::endl;
		}

		uidtracker::~uidtracker()
		{
			std::cerr << "Debug: destroying uid " << _uid << std::endl;
		}

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
		};

		void timestamp(bool show, std::ostream &out)
		{
			static time_t stamp = 0; // Set to zero the first time
			time_t newstamp = time (NULL);
			if (show) out << "Timestamp Delta: " << (newstamp - stamp) << " seconds" << std::endl;
			stamp = newstamp;
		}

		unsigned int getuniqueobj(const char* file, int line, const char* func)
		{
			keymap temp(file,line,func);

			if (!UNIQUE_KEYS.count(temp))
			{
				UNIQUE_KEYS[temp] = 0;
			} else {
				UNIQUE_KEYS[temp] = UNIQUE_KEYS[temp] + 1;
			}
			return UNIQUE_KEYS[temp];
		}

		void listuniqueobj(std::ostream &out, bool showifnone)
		{
			// This is a debugging function that will output the keymaps, counts and identities
			using namespace std;
			if (showifnone || UNIQUE_KEYS.size())
			{
				out << "\nUnique object keymap listing:\n";
				out << UNIQUE_KEYS.size() << " objects\n";
				if (UNIQUE_KEYS.size())
				{
					out << "File - Function - Line - Count\n";
					for (map<keymap,int>::iterator it = UNIQUE_KEYS.begin(); it != UNIQUE_KEYS.end(); it++)
					{
						out << it->first.file << " - " << it->first.function << " - " << it->first.line << " - " << it->second << endl;

					}
				}
				out << endl;
			}
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

		// Instance debugging
		namespace instances
		{
			std::map<std::string, void*> _instances;
			void registerInstance(const char* id, void* obj)
			{
				_instances.insert( std::pair<std::string, void*>(std::string(id), obj));
			}

			void freeInstances()
			{
				// TODO: check if instance is already freed
				for (auto it = _instances.begin(); it != _instances.end(); it++)
					delete (it->second);
			}
		}

	}; // end debug
}; // end rtmath

// These go outside of rtmath
/*
#ifdef HEAP_CHECK

typedef struct {
	void*	address;
	size_t	size;
	char	file[64];
	int	line;
} ALLOC_INFO;

typedef std::set<ALLOC_INFO*> AllocList;

AllocList *allocList;

void AddTrack(void* addr,  size_t asize,  const char *fname, int lnum)
{
	ALLOC_INFO *info;

	if(!allocList) {
		allocList = new(AllocList);
	}

	info = new(ALLOC_INFO);
	info->address = addr;
	strncpy(info->file, fname, 63);
	info->line = lnum;
	info->size = asize;
	allocList->insert(allocList->begin(), info);
};

void RemoveTrack(void* addr)
{
	AllocList::iterator i;

	if(!allocList)
		return;
	for(i = allocList->begin(); i != allocList->end(); i++)
	{
		if((*i)->address == addr)
		{
			allocList->erase((*i));
			break;
		}
	}
};

void DumpUnfreed()
{
	AllocList::iterator i;
	int totalSize = 0;
	char buf[1024];

	if(!allocList)
		return;

	for(i = allocList->begin(); i != allocList->end(); i++) {
		sprintf(buf, "%-50s:\t\tLINE %d,\t\tADDRESS %d\t%d unfreed\n",
			(*i)->file, (*i)->line, (*i)->address, (*i)->size);
		OutputDebugString(buf);
		totalSize += (*i)->size;
	}
	sprintf(buf, "-----------------------------------------------------------\n");
	OutputDebugString(buf);
	sprintf(buf, "Total Unfreed: %d bytes\n", totalSize);
	OutputDebugString(buf);
};
#endif
*/
