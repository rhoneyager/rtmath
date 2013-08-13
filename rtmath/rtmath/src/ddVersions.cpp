#pragma warning( disable : 4996 ) // -D_SCL_SECURE_NO_WARNINGS


#include <boost/thread/mutex.hpp>
//#include <boost/thread/lock_guard.hpp>
#include <boost/thread/locks.hpp>
//#include <boost/bimap.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <iostream>
#include <sstream>
#include <vector>
#include "../rtmath/ddscat/ddVersions.h"
#include "../rtmath/error/error.h"

namespace
{
	bool verMapInited = false;
	boost::mutex verMapMutex;
	typedef std::map< size_t, std::string > verMapType;
	verMapType avgMap;
	void initVerMap()
	{
		boost::lock_guard<boost::mutex> guard(verMapMutex);
		if (verMapInited) return;
		verMapInited = true;

		avgMap.insert( verMapType::value_type(71, "DDSCAT 7.0 [08.09.03]" ) );
		avgMap.insert( verMapType::value_type(707, "DDSCAT 7.0.7 [09.12.11]" ) );
		avgMap.insert( verMapType::value_type(708, "DDSCAT 7.0.8 [10.02.04]" ) );
		avgMap.insert( verMapType::value_type(71, "DDSCAT 7.1 [10.02.07]" ) );
		avgMap.insert( verMapType::value_type(710, "DDSCAT 7.1.0 [10.03.03]" ) );
		avgMap.insert( verMapType::value_type(720, "DDSCAT 7.2.0 [12.02.16]" ) );
		avgMap.insert( verMapType::value_type(721, "DDSCAT 7.2.1 [12.05.14]" ) );
		avgMap.insert( verMapType::value_type(722, "DDSCAT 7.2.2 [12.06.05]" ) );
		avgMap.insert( verMapType::value_type(73, "DDSCAT 7.3 [13.04.04]-BETA" ) );
		avgMap.insert( verMapType::value_type(730, "DDSCAT 7.3.0 [13.05.03]" ) );
	}
}

namespace rtmath
{
	namespace ddscat
	{
		namespace ddVersions
		{
			size_t getVerId(const std::string& s)
			{
				//initVerMap();
				// Use boost::regex to find the 0.0.0 matching string,
				// then expand it.
				std::string::const_iterator start, end;
				start = s.begin();
				end = s.end();

				static const boost::regex e("[0-9]\\.[0-9]\\.[0-9]");
				static const boost::regex eb("[0-9]\\.[0-9]");
				boost::match_results<std::string::const_iterator> what;
				boost::match_flag_type flags = boost::match_default;
				if (!boost::regex_search(s, what, e, flags))
					if (!boost::regex_search(s,what,eb, flags)) 
						throw rtmath::debug::xBadInput("Bad version id");
				// what.begin() now contains the raw 0.0.0 version text. Split it.
				using namespace std;
				using namespace boost;
				vector<string> tokens;
				string sss = what.begin()->str();
				split(tokens, sss, is_any_of("."), token_compress_on);

				size_t ret = 0;
				for (auto it = tokens.begin(); it != tokens.end(); ++it)
				{
					//                                std::cerr << "Token " << *it << std::endl;
					size_t i = atoi(it->c_str());
					ret *= 10;
					ret += i;
				}

				return ret;
			}

			std::string getVerString(size_t id)
			{
				//initVerMap();
				std::string s;
				std::ostringstream out;
				std::vector<size_t> v;
				// Just progressively chop off the end figures
				do
				{
					size_t i = id % 10;
					v.push_back(i);
					id = id / 10;
				} while (id);

				for (auto it = v.rbegin(); it != v.rend(); ++it)
				{
					if (it != v.rbegin()) out << ".";
					out << *it;
				}

				s = out.str();
				return s;
			}

			std::string getVerAvgHeaderString(size_t id)
			{
				initVerMap();
				std::string s;
				if (avgMap.count(id)) s = avgMap.at(id);
				else throw rtmath::debug::xBadInput("getVerAvgHeaderString id not found");
				return s;
			}

			bool isVerWithin(size_t ver, size_t min, size_t max)
			{
				// Needed because of the obtuse ddscat versioning scheme
				// ex: 7.2.0 is the first release after 7.2

				// min is inclusive, max is exclusive. If zero is passed to max, then
				// there is no upper bound.

				if (ver < 100)
				{
					bool maxM = false; // Flag set when upper version > 100 (like 7.2.0)
					if (min > 100) min /= 10;
					if (max > 100) { max /= 10; maxM = true; }
					if (ver < min) return false;
					if (!max) return true;
					if (ver > max) return false;
					if (ver != max) return true;
					if (maxM) return true; // 7.2 < 7.2.0 case
					return false;
				} else { // ver has three digits
					if (min < 100) min *= 10;
					if (max < 100) max *= 10;

					if (ver < min) return false; // min 2-digit case handled above
					if (!max) return true;
					if (ver > max) return false;
					return true; // ver is in [0,max).
				}
			}

			bool isVerWithin(size_t ver, const std::string &range)
			{
				// range represents several intervals, and if ver is within 
				// the union of these intervals, the expression evaluates to true.
				// each range interval is comma-delimeted. the interval itself 
				// is expressed using : notation. 

				using namespace std;
				// Prepare tokenizer
				typedef boost::tokenizer<boost::char_separator<char> >
					tokenizer;
				boost::char_separator<char> sep(",");
				boost::char_separator<char> seprange(":");
				tokenizer tcom(range, sep);
				for (auto ot = tcom.begin(); ot != tcom.end(); ++ot)
				{
					tokenizer trange(*ot,seprange);
					vector<size_t> range;
					size_t i = 0;
					for (auto rt = trange.begin(); rt != trange.end(); rt++, i++)
					{
						try {
							string s = *rt;
							boost::algorithm::trim(s);
							if (i < 3)
							{
								range.push_back(boost::lexical_cast<size_t>(s));
							}
						}
						catch (...)
						{
							throw rtmath::debug::xBadInput(rt->c_str());
						}
					}

					if (!range.size()) continue;
					size_t min = *range.begin();
					size_t max = *range.rbegin();

					if (isVerWithin(ver, min, max)) return true;
				}

				return false;
			}

			size_t getDefaultVer()
			{
#pragma message("Warning: ddVersions needs a full getDefaultVer() implementation")
				return 722;
			}
		}
	}
}
