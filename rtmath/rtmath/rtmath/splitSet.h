#pragma once
#include "defs.h"
#include <map>
#include <set>
#include <string>
#include <vector>

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

		/// Shortcut that already passes parsed information
		template <class T>
		void splitSet(
			const T &start, const T &end, const T &interval,
			const std::string &specializer,
			std::set<T> &expanded);

		/** \brief Extracts information from interval notation
		*
		**/
		template <class T>
		void extractInterval(
			const std::string &instr,
			T &start, T &end, T &interval, size_t &num,
			std::string &specializer);



		/** \brief Convenience function to split a null-separated string list into a vector of strings.
		*
		* Commonly-used to split up the results of a Ryan_Debug::ProcessInfo command-line structure.
		**/
		void DLEXPORT_rtmath_core splitNullVector(
			const std::string &instr, std::vector<std::string> &out);

		/** \brief Convenience function to split a null-separated string list into a map of strings.
		*
		* Commonly-used to split up the results of a Ryan_Debug::ProcessInfo environment structure.
		**/
		void DLEXPORT_rtmath_core splitNullMap(
			const std::string &instr, std::map<std::string, std::string> &out);

	}
}

