#pragma once
#include "defs.h"
#include <map>
#include <set>
#include <string>

namespace rtmath {
	namespace config {
		/**
		* \brief Function that expands sets of _numbers_ with 
		* separators of commas, dashes and colons.
		*
		* \todo Check template export on msvc
		**/
		template <class T>
		void splitSet(
			const std::string &instr, 
			std::set<T> &expanded,
			const std::map<std::string, 
			std::string> *aliases = nullptr);

		/// \brief Specialization for splitting strings. These 
		/// objects have no ranges to be compared against.
		template <> void DLEXPORT_rtmath_core splitSet<std::string>(
			const std::string &instr, 
			std::set<std::string> &expanded,
			const std::map<std::string, 
			std::string> *aliases);

	}
}

