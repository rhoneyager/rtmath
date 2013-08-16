/* debug.h - The debugging system for the library */
//#pragma once



#pragma once

#include <iostream>
#include <string>
#include "error.h"

namespace boost { namespace program_options { 
	class options_description; class variables_map; } }

namespace rtmath
{
	namespace debug
	{
		/// Generate the startup message giving library information
		void debug_preamble(std::ostream &out = std::cerr);
		/// Get revision of the code
		int rev(void);
		/// Write the last recorded code position. Used in error throw messages.
		void dumpErrorLocation(std::ostream &out = std::cerr);

		/**
		* \brief Adds options to a program
		*
		* \item cmdline provides options only allowed on the command line
		* \item config provides options available on the command line and in a config file
		* \item hidden provides options allowed anywhere, but are not displayed to the user
		**/
		void add_options(
			boost::program_options::options_description &cmdline,
			boost::program_options::options_description &config,
			boost::program_options::options_description &hidden);
		/// Processes static options defined in add_options
		/// \todo Add processor for non-static options
		void process_static_options(
			boost::program_options::variables_map &vm);

		/// \brief Class used in counting number of times a certain code position 
		/// was passed.
		/// \deprecated keymap is no longer used
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

	/// \def UNIMPLEMENTED() Macro to throw unimplemented function error with function signature
#ifdef __GNUC__
#define UNIMPLEMENTED() { throw ::rtmath::debug::xUnimplementedFunction(__PRETTY_FUNCTION__); };
#endif
#ifdef _MSC_FULL_VER
#define UNIMPLEMENTED() { throw ::rtmath::debug::xUnimplementedFunction(__FUNCSIG__); };
#endif

} // end namespace rtmath
