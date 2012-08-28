#include "../rtmath/Stdafx.h"
#include <algorithm>
#include <string>
#include <sstream>
#include <map>
#include <vector>
#include <set>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <boost/math/constants/constants.hpp>
#include <boost/tokenizer.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>

#include "../rtmath/splitSet.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"

namespace rtmath {
	namespace config {

		template <class T>
		void splitSet(const std::string &instr, std::set<T> &expanded,
			const std::map<std::string, std::string> *aliases)
		{
			using namespace std;
			// Prepare tokenizer
			typedef boost::tokenizer<boost::char_separator<char> >
				tokenizer;
			boost::char_separator<char> sep(",");
			boost::char_separator<char> seprange(":");
			{
				tokenizer tcom(instr,sep);
				for (auto ot = tcom.begin(); ot != tcom.end(); ot++)
				{
					// At this junction, do any alias substitution

					std::string ssubst;

					std::map<std::string, std::string> defaliases;
					if (!aliases) aliases = &defaliases; // Provides a convenient default

					if (aliases->count(*ot))
					{
						ssubst = aliases->at(*ot);
						// Recursively call splitSet to handle bundles of aliases
						splitSet<T>(ssubst, expanded, aliases);
					} else { 
						// Separated based on commas. Expand for dashes and colons
						// TODO: allow string specifier after three params
						tokenizer trange(*ot,seprange);
						vector<T> range;
						string specializer;
						size_t i = 0;
						for (auto rt = trange.begin(); rt != trange.end(); rt++, i++)
						{
							try {
								string s = *rt;
								boost::algorithm::trim(s);
								if (i < 3)
								{
									range.push_back(boost::lexical_cast<T>(s));
								} else {
									specializer = s;
								}
							}
							catch (...)
							{
								throw rtmath::debug::xBadInput(rt->c_str());
							}
						}
						// Look at range. If one element, just add it. If two or 
						// three, calculate the inclusive interval
						if (range.size() == 1)
						{
							if (expanded.count(range[0]) == 0)
								expanded.insert(range[0]);
						} else {
							T start, end = 0, interval = 1;
							start = range[0];
							end = range[range.size()-1];
							if (range.size() > 2) interval = range[1];
							std::transform(specializer.begin(),specializer.end(),specializer.begin(),::tolower);
							if (specializer == "")
							{
								if (start < 0 || end < 0 || start > end || interval < 0)
								{
									// Die from invalid range
									// Should really throw error
									throw rtmath::debug::xBadInput(ot->c_str());
									//cerr << "Invalid range " << *ot << endl;
									//exit(1);
								}
								for (T j=start;j<=end+(interval/100.0);j+=interval)
								{
									if (expanded.count(j) == 0)
										expanded.insert(j);
								}
							} else if (specializer == "lin") {
								// Linear spacing
								double increment = (end - start) / (interval); // so interval of 1 gives midpoint
								if (!increment) expanded.insert(start);
								for (T j=start+(increment/2.0); j<end+(increment/100.0);j+=increment)
								{
									if (expanded.count(j) == 0)
										expanded.insert(j);
								}
							} else if (specializer == "log") {
								if (start == 0 || end == 0)
									throw rtmath::debug::xBadInput("Cannot take inverse of zero.");
								double is = log10(start); 
								double ie = log10(end); 
								double increment = (ie - is) / (interval);
								if (!increment) expanded.insert(start);
								for (T j=is+(increment/2.0); j<ie+(increment/100.0);j+=increment)
								{
									T k = (T) pow((T) 10.0,(j));
									if (expanded.count(k) == 0)
										expanded.insert(k);
								}
							} else if (specializer == "inv") {
								if (start == 0 || end == 0)
									throw rtmath::debug::xBadInput("Cannot take inverse of zero.");
								T is = 1.0 / start; 
								T ie = 1.0 / end; 
								T increment = (is - ie) / (interval);
								if (!increment) expanded.insert(start);
								for (T j=ie+(increment/2.0); j<is+(increment/100.0);j+=increment)
								{
									T k = ((T) 1.0) / j;
									if (expanded.count(k) == 0)
										expanded.insert(k);
								}
							} else if (specializer == "cos") {
								// Linear in cos
								// start, end are in degrees
								const double pi = boost::math::constants::pi<double>();
								double cs = cos(start * pi / 180.0);
								double ce = cos(end * pi / 180.0);
								double increment = (ce - cs) / (interval);
								if (increment == 0) expanded.insert(start);
								if (increment < 0)
								{
									increment *= -1.0;
									std::swap(cs,ce);
								}
								for (T j=cs+(increment/2.0); j<ce+(increment/100.0);j+=increment)
								{
									T k = (T) (acos(j) * 180.0 / pi);
									if (expanded.count(k) == 0)
										expanded.insert(k);
								}
							} else {
								throw rtmath::debug::xBadInput(ot->c_str());
							}
						}
					}
				}
			}
		}

		template <> void splitSet<std::string>(const std::string &instr, std::set<std::string> &expanded,
			const std::map<std::string, std::string> *aliases)
		{
			using namespace std;
			// Prepare tokenizer
			typedef boost::tokenizer<boost::char_separator<char> >
				tokenizer;
			boost::char_separator<char> sep(",");

			std::string ssubst;

			std::map<std::string, std::string> defaliases;
			if (!aliases) aliases = &defaliases; // Provides a convenient default

			tokenizer tcom(instr,sep);
			for (auto ot = tcom.begin(); ot != tcom.end(); ot++)
			{
				if (aliases->count(*ot))
				{
					ssubst = aliases->at(*ot);
					// Recursively call splitSet to handle bundles of aliases
					splitSet<std::string>(ssubst, expanded, aliases);
				} else { 
					if (expanded.count(*ot) == 0)
						expanded.insert(*ot);
				}
			}
		}

#define SPEC_SPLITSET(T) \
	template <> void splitSet<T>(const std::string &instr, std::set<T> &expanded, \
		const std::map<std::string, std::string> *aliases);

		SPEC_SPLITSET(int);
		SPEC_SPLITSET(size_t);
		SPEC_SPLITSET(float);
		SPEC_SPLITSET(double);

	}; // end namespace config
}; // end namespace rtmath


