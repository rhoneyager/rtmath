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

		/// \brief Extracts information from interval notation
		template <class T>
		void extractInterval(
			const std::string &instr,
			T &start, T &end, T &interval, size_t &num,
			std::string &specializer);



		/** \brief Convenience function to split a null-separated string list into a vector of strings.
		*
		* Commonly-used to split up the results of a Ryan_Debug::ProcessInfo command-line structure.
		**/
		void DLEXPORT_rtmath_core splitVector(
			const std::string &instr, std::vector<std::string> &out, char delim = '\0');
		inline void DLEXPORT_rtmath_core splitNullVector(
			const std::string &instr, std::vector<std::string> &out) { splitVector(instr, out); }

		/** \brief Convenience function to split a null-separated string list into a map of strings.
		*
		* Commonly-used to split up the results of a Ryan_Debug::ProcessInfo environment structure.
		**/
		void DLEXPORT_rtmath_core splitNullMap(
			const std::string &instr, std::map<std::string, std::string> &out);


		/** \brief Class to define and search on intervals.
		*
		* This is a simple class used for searching based on user input. It does not 
		* provide interval unions, intersections, etc. It can, however, aid in setting 
		* these up, such as for a database query.
		* 
		* Accepts standard paramSet notation, but also adds the '-' range operator, 
		* implying that values may be found in a certain range.
		**/
		template <class T>
		class intervals
		{
		public:
			std::vector<std::pair<T, T> > ranges;
			intervals(const std::string &s = "") { if (s.size()) append(s); }
			intervals(const std::vector<std::string> &s) { append(s); }
			~intervals() {}
			void append(const std::string &instr,
				const std::map<std::string, std::string> *aliases = nullptr)
			{
				std::vector<std::string> splits;
				splitVector(instr, splits, ',');
				std::set<T> vals;
				for (const auto &s : splits)
				{
					std::map<std::string, std::string> defaliases;
					if (!aliases) aliases = &defaliases; // Provides a convenient default

					if (aliases->count(s))
					{
						std::string ssubst = aliases->at(s);
						// Recursively call splitSet to handle bundles of aliases
						append(ssubst, aliases);
					} else {
						T start, end, interval;
						size_t n;
						std::string specializer;
						bool isRange = false;
						extractInterval(s, start, end, interval, n, specializer);
						if (specializer == "range")
						{
							ranges.push_back(std::pair<T, T>(start, end));
						} else {
							splitSet(start, end, interval, specializer, vals);
						}
					}
				}
				for (const auto &v : vals)
				{
					ranges.push_back(std::pair<T, T>(v, v));
				}
			}
			void append(const std::vector<std::string> &s,
				const std::map<std::string, std::string> *aliases = nullptr)
			{
				for (const auto &str : s) append(str, aliases);
			}
			void append(const intervals<T>& src)
			{
				ranges.insert(ranges.end(), src.ranges.begin(), src.ranges.end());
			}
			bool inRange(const T& val) const
			{
				for (const auto &r : ranges)
				{
					if (val >= r.first && val < r.second) return true;
				}
				return false;
			}
			bool isNear(const T& val, const T& linSep, const T& factorSep) const
			{
				for (const auto &r : ranges)
				{
					T lower = (r.first * (static_cast<T>(1) - factorSep)) - linSep,
						upper = (r.second * (static_cast<T>(1) + factorSep)) + linSep;
					if (val >= lower && val < upper) return true;
				}
				return false;
			}
		};
		extern template class intervals < int >;
		extern template class intervals < double >;
		extern template class intervals < float >;
		extern template class intervals < unsigned int >;
		extern template class intervals < long >;
		extern template class intervals < unsigned long >;
	}
}

