/* debug.h - The debugging system for the library */
//#pragma once



#pragma once

#include <iostream>
#include <string>
#include "error.h"


namespace rtmath
{
	namespace debug
	{
		// Generate the startup message giving 
		// library information
		void debug_preamble(std::ostream &out = std::cerr);
		int rev(void);
		void dumpErrorLocation(std::ostream &out = std::cerr);
		struct keymap
		{
		public:
			std::string file;
			unsigned int line;
			std::string function;
			keymap() {};
			keymap(const char* file, int line, const char* func)
			{
				this->file = std::string(file);
				this->line = line;
				this->function = std::string(func);
			}
			bool operator< (const keymap &rhs) const
			{
				int a = file.compare(rhs.file);
				if (!a)
				{
					if (a>0) return true;
					return false;
				}
				return line < rhs.line;
			}
		};

	} // end namespace debug

// Throw unimplemented function

#ifdef __GNUC__
#define UNIMPLEMENTED() { throw ::rtmath::debug::xUnimplementedFunction(__PRETTY_FUNCTION__); };
#endif
#ifdef _MSC_FULL_VER
#define UNIMPLEMENTED() { throw ::rtmath::debug::xUnimplementedFunction(__FUNCSIG__); };
#endif

// Die on error
#ifdef __GNUC__
#define DIE() { throw ::rtmath::debug::diemsg(__FILE__,__LINE__,__PRETTY_FUNCTION__); };
#endif
#ifdef _MSC_FULL_VER
#define DIE() { throw ::rtmath::debug::diemsg(__FILE__,__LINE__,__FUNCSIG__); };
#endif

} // end namespace rtmath

