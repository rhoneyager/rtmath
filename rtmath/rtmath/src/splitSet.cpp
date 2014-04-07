#include "Stdafx-core.h"
#include <algorithm>
#include <string>
#include <map>
#include <vector>
#include <set>
#include <boost/math/constants/constants.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>

#include "../rtmath/splitSet.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"

#pragma warning( disable : 4244 ) // lots of template math involves doubles, and I'm sick of static casting

namespace rtmath {
	namespace config {

		template <class T>
		void splitSet(
			const T &Tstart, const T &Tend, const T &Tinterval,
			const std::string &Tspecializer,
			std::set<T> &expanded)
		{
			using namespace std;
			double start, end, interval;
			start = boost::lexical_cast<double>(Tstart);
			end = boost::lexical_cast<double>(Tend);
			interval = boost::lexical_cast<double>(Tinterval);
			std::string specializer(Tspecializer);
			std::transform(specializer.begin(),specializer.end(),specializer.begin(),::tolower);
			if (specializer == "")
			{
				if ((start > end && interval > 0) || (start < end && interval < 0))
				{
					// Die from invalid range
					// Should really RTthrow error
					throw rtmath::debug::xBadInput("Invalid range");
				}
				for (double j=start;j<=end+(interval/100.0);j+=interval)
				{
					if (expanded.count((T) j) == 0)
						expanded.insert((T) j);
				}
			} else if (specializer == "lin") {
				// Linear spacing
				double increment = (end - start) / (interval); // so interval of 1 gives midpoint
				if (!increment) expanded.insert((T) start);
				for (double j=start+(increment/2.0); j<end+(increment/100.0);j+=increment)
				{
					if (expanded.count((T) j) == 0)
						expanded.insert((T) j);
				}
			} else if (specializer == "log") {
				if (start == 0 || end == 0)
					throw rtmath::debug::xBadInput("Cannot take inverse of zero.");
				double is = log10( (double) start); 
				double ie = log10( (double) end); 
				double increment = (ie - is) / (interval);
				if (!increment) expanded.insert((T) start);
				for (double j=is+(increment/2.0); j<ie+(increment/100.0);j+=increment)
				{
					double k = pow((double) 10.0, (double) j);
					if (expanded.count((T) k) == 0)
						expanded.insert((T) k);
				}
			} else if (specializer == "inv") {
				if (start == 0 || end == 0)
					throw rtmath::debug::xBadInput("Cannot take inverse of zero.");
				double is = 1.0 / start; 
				double ie = 1.0 / end; 
				double increment = (is - ie) / (interval);
				if (!increment) expanded.insert((T) start);
				for (double j=ie+(increment/2.0); j<is+(increment/100.0);j+=increment)
				{
					double k = (1.0) / j;
					if (expanded.count((T) k) == 0)
						expanded.insert((T) k);
				}
			} else if (specializer == "cos") {
				// Linear in cos
				// start, end are in degrees
				const double pi = boost::math::constants::pi<double>();
				int ai = (int) (interval) % 2;
				double cs = cos(start * pi / 180.0);
				double ce = cos(end * pi / 180.0);
				double increment = (ai) ? (ce - cs) / (interval-1) : (ce - cs) / (interval);
				if (increment == 0) expanded.insert(start);
				if (increment != increment) // nan check - occurs when selecting only one value, and bounds are the same
				{
					expanded.insert(start);
					return;
				}
				if (increment < 0)
				{
					increment *= -1.0;
					std::swap(cs,ce);
				}
				// For even n, divide into intervals and use the midpoint of the interval.
				// For odd n, use the endpoints. Note that the weights for orientations 
				// (not computed here) will be different for the two choices.
				if (!ai) cs += increment/2.0;
				for (double j=cs; j<ce+(increment/10.0);j+=increment)
				{
					// max and min to avoid j>1 and j<-1 error from rounding
					double k = (acos((double) max(min(j,1.0),-1.0)) * 180.0 / pi);
					if (expanded.count((T) k) == 0)
						expanded.insert((T) k);
				}
			} else {
				throw rtmath::debug::xBadInput("Bad input in inner splitSet");
			}
		}

#define SPEC_SPLITSET_A(T) \
	template void DLEXPORT_rtmath_core splitSet<T>(const T&, const T&, \
	const T&, const std::string&, std::set<T> &);

		SPEC_SPLITSET_A(int);
		SPEC_SPLITSET_A(size_t);
		SPEC_SPLITSET_A(float);
		SPEC_SPLITSET_A(double);


		template <class T>
		void extractInterval(
			const std::string &instr,
			T &start, T &end, T &interval, size_t &num, std::string &specializer)
		{
			using namespace std;
			// Prepare tokenizer
			typedef boost::tokenizer<boost::char_separator<char> >
				tokenizer;
			boost::char_separator<char> seprange(":");
			tokenizer trange(instr,seprange);
			vector<T> range;
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
				start = range[0];
				end = range[0];
				interval = 0;
				num = 1;
			} else {
				start = range[0];
				end = range[range.size()-1];
				interval = 0;
				if (specializer.size())
				{
					if (range.size() > 2) num = (size_t) range[1];
				} else {
					if (range.size() > 2) interval = range[1];
					// Linear spacing, starting at start.
					num = (size_t) ( ( (end - start) / interval) + 1);
				}
			}
		}

#define SPEC_SPLITSET_INTERVAL(T) \
	template void DLEXPORT_rtmath_core extractInterval<T>( \
	const std::string&, T&, T&, T&, size_t&, std::string&);

		SPEC_SPLITSET_INTERVAL(int);
		SPEC_SPLITSET_INTERVAL(size_t);
		SPEC_SPLITSET_INTERVAL(float);
		SPEC_SPLITSET_INTERVAL(double);



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
							double start, end = 0, interval = 1;
							start = (double) range[0];
							end = (double) range[range.size()-1];
							if (range.size() > 2) interval = (double) range[1];

							// I'm moving the logic to the other template definition, as it 
							// let's me split stuff without casts back to strings.
							splitSet<T>(start, end, interval, specializer, expanded);
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
			boost::char_separator<char> sep(",;");

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
	template void DLEXPORT_rtmath_core splitSet<T>(const std::string &instr, std::set<T> &expanded, \
	const std::map<std::string, std::string> *aliases);

		SPEC_SPLITSET(int);
		SPEC_SPLITSET(size_t);
		SPEC_SPLITSET(float);
		SPEC_SPLITSET(double);




		void DLEXPORT_rtmath_core splitVector(
			const std::string &instr, std::vector<std::string> &out, char delim)
		{
			using namespace std;
			out.clear();
			if (!instr.size()) return;

			// Fast string splitting based on null values.
			const char* start = instr.data();
			const char* stop = instr.data() + instr.size();
			while (start < stop)
			{
				// Find the next null character
				const char* sep = start;
				sep = std::find(start, stop, delim);
				if (*start == delim)
				{
					start = sep+1;
					continue;
				}
				out.push_back(std::string(start, sep));
				start = sep+1;
			}
		}

		void DLEXPORT_rtmath_core splitNullMap(
			const std::string &instr, std::map<std::string, std::string> &out)
		{
			using namespace std;
			out.clear();
			if (!instr.size()) return;

			// Fast string splitting based on null values.
			const char* start = instr.data();
			const char* stop = instr.data() + instr.size();
			while (start < stop)
			{
				// Find the next null character
				const char* sep = start;
				sep = std::find(start, stop, '\0');
				if (*start == '\0')
				{
					start = sep+1;
					continue;
				}
				// Split based on location of equal sign
				//out.push_back(std::string(start, sep - 1));
				const char* sepc = std::find(start, sep, '=');
				// If the = cannot be found, then it is a key with an empty value.
				std::string key(start, sepc);
				if (!key.size())
				{
					start = sep+1;
					continue;
				}
				/*
				if (//sepc+1 >= stop || 
					//sep > stop || 
					sepc+1>sep)
				{
					std::cerr << "Pointer error: sepc " << &sepc << " sep " << &sep
						<< " stop " << &stop << " start " << &start << std::endl;
					throw;
				}
				*/
				std::string val;
				if (sepc < sep)
					val = std::string(sepc + 1, sep);
				out.insert(std::make_pair(key, val));
				start = sep+1;
			}
		}

	}
}


