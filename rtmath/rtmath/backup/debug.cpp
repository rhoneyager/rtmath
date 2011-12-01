/* debug.cpp
* The debugging file, where all of the error-handling
* and versioning code resides.
*/

#include "Stdafx.h"
#include "debug.h"
#include "error.h"

#include "debug_mem.h"

// This file just defines the subversion revision, created at a pre-build strp
#ifdef _WIN32
#ifdef _MSC_FULL_VER
#include "debug_subversion.h"
#endif
#else

//#include "debug_subversion.svn.h"
#endif
namespace rtmath
{
	namespace debug
	{
		std::map<keymap*, int> UNIQUE_KEYS;
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
#ifdef SUB_REV
			std::cerr << "SVN Revision " << SUB_REV << std::endl;
			std::cerr << "SVN Revision Date: " << SUB_DATE << std::endl;
			std::cerr << "SVN Working Copy Range: " << SUB_WCRANGE << std::endl;
			std::cerr << "SVN Source: " << SUB_SOURCE << std::endl;
#else
			std::cerr << "SVN Revision Unknown" << std::endl;
#endif
#ifdef _DEBUG
			std::cerr << "Debug Version" << std::endl;
#else
			std::cerr << "Release Version" << std::endl;
#endif
#ifdef _OPENMP
			std::cerr << "OpenMP Supported" << std::endl;
#else
			std::cerr << "OpenMP Disabled" << std::endl;
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
#ifdef __PATHCC__
			std::cerr << "EKOPath Compiler " << __PATHCC__ << "." << __PATHCC_MINOR__ << "." << __PATHCC_PATCHLEVEL__ << std::endl;
#endif
#ifdef __llvm__
			std::cerr << "LLVM Compiler Suite" << std::endl;
#endif
#ifdef __clang__
			std::cerr << " clang " << __clang_major__ << "." << __clang_minor__ << "." << __clang_patchlevel__ << std::endl;
#endif
#ifdef __INTEL_COMPILER
			std::cerr << "Intel Compiler Version " << __INTEL_COMPILER << std::endl;
			std::cerr << " Compiler build date: " << __INTEL_COMPILER_BUILD_DATE << std::endl;
#endif
#ifdef _MSC_FULL_VER
			std::cerr << "Microsoft Visual Studio Compiler Version " << _MSC_FULL_VER << std::endl;
#endif
			std::cerr << std::endl;
		};

		void timestamp(bool show)
		{
			static time_t stamp = 0; // Set to zero the first time
			time_t newstamp = time (NULL);
			if (show) std::cerr << "Timestamp Delta: " << (newstamp - stamp) << " seconds" << std::endl;
			stamp = newstamp;
		}

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
