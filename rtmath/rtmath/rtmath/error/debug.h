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
		void timestamp(bool show = false, std::ostream &out = std::cerr);
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
		void DEBUG_TIMESTAMP(void);
		unsigned int getuniqueobj(const char* file, int line, const char* func);
		void listuniqueobj(std::ostream &out = std::cerr, bool showifnone = true);

		class uidtracker
		{
		public:
			uidtracker();
			virtual ~uidtracker();
			inline unsigned int uid() { return _uid; }
		private:
			unsigned int _uid;
			static unsigned int _uidcount;
		};

		// rtmath-apps application entry point. Not fully necessary, but it sets
		// several really convenient settings.
		void appEntry(int argc, char** argv); // in os_functions.cpp
		bool waitOnExit(); // in os_functions.cpp
		void appExit(); // Convenient default atexit function
		bool pidExists(int pid);
		// App warning functions
		//void warn(const char* msg);
	}; // end namespace debug

	// see assert.h for how this is made

#ifdef __GNUC__
#define GETOBJKEY() rtmath::debug::getuniqueobj(__FILE__, __LINE__, __PRETTY_FUNCTION__)
#endif
#ifdef _MSC_FULL_VER
#define GETOBJKEY() rtmath::debug::getuniqueobj(__FILE__, __LINE__, __FUNCSIG__)
#endif

#ifdef _DEBUG

#ifdef __GNUC__
#define MARK() {std::cerr << "MARK " << rtmath::debug::getuniqueobj("DEBUG", 0, "MARK") << ": " << __FILE__ << ":" << __LINE__ << ": " << __PRETTY_FUNCTION__ << "\n";};
#define MARKUID() {std::cerr << "MARK: " << __FILE__ << ":" << __LINE__ << ": " << __PRETTY_FUNCTION__ << ": " << this->_uid << "\n";};
#define MARKFUNC() {std::cerr << "MARK: " << __FILE__ << ":" << __LINE__ << ": " << __PRETTY_FUNCTION__ << "\n";};
#endif
#ifdef _MSC_FULL_VER
#define MARK() {std::cerr << "MARK " << rtmath::debug::getuniqueobj("DEBUG", 0, "MARK") << ": " << __FILE__ << ":" << __LINE__ << ": " << __FUNCSIG__ << "\n";};
#define MARKUID() {std::cerr << "MARK: " << __FILE__ << ":" << __LINE__ << ": " << __FUNCSIG__ << ": " << this->_uid << "\n";};
#define MARKFUNC() {std::cerr << "MARK: " << __FILE__ << ":" << __LINE__ << ": " << __FUNCSIG__ << "\n";};
#endif

#else
#define MARK() { rtmath::debug::getuniqueobj("DEBUG", 0, "MARK"); };
#define MARKUID() { rtmath::debug::getuniqueobj("DEBUG", 0, "MARKUID"); };
#define MARKFUNC() { rtmath::debug::getuniqueobj("DEBUG", 0, "MARKFUNC"); };
#endif

// Throw unimplemented function

#ifdef __GNUC__
#define UNIMPLEMENTED() { throw rtmath::debug::xUnimplementedFunction(__PRETTY_FUNCTION__); };
#endif
#ifdef _MSC_FULL_VER
#define UNIMPLEMENTED() { throw rtmath::debug::xUnimplementedFunction(__FUNCSIG__); };
#endif

// Die on error
#ifdef __GNUC__
#define DIE() { throw rtmath::debug::diemsg(__FILE__,__LINE__,__PRETTY_FUNCTION__); };
#endif
#ifdef _MSC_FULL_VER
#define DIE() { throw rtmath::debug::diemsg(__FILE__,__LINE__,__FUNCSIG__); };
#endif

}; // end namespace rtmath

