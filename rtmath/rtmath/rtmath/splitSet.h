#pragma once
#include <map>
#include <set>
#include <string>

namespace rtmath {
	namespace config {
		// Function that expands sets of _numbers_ with 
		// separators of commas, dashes and colons.
		template <class T>
		void splitSet(
			const std::string &instr, 
			std::set<T> &expanded,
			const std::map<std::string, 
			std::string> *aliases = nullptr);

		// Specialization for splitting strings. These 
		// objects have no ranges to be compared against.
		template <> void splitSet<std::string>(
			const std::string &instr, 
			std::set<std::string> &expanded,
			const std::map<std::string, 
			std::string> *aliases);

	}
}

