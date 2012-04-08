#pragma once
#include <string>
#include <boost/tokenizer.hpp>
#include <vector>
#include <map>
#include <set>

namespace rtmath {
	namespace parsers {


		// These are no longer necessary, but I'll save them as examples and may move them elsewhere
		// TODO: possibly merge these with command-line parsing functions, as I am 
		// duplicating much of the functionality here.

		template <class T> bool pNumeric(const std::string &in, std::vector<T> &vals)
		{
			vals.clear();
			// Parse based on spaces
			typedef boost::tokenizer<boost::char_separator<char> >
				tokenizer;
			boost::char_separator<char> sep(" \t");
			tokenizer tcom(in,sep);
			for (auto it=tcom.begin(); it != tcom.end(); ++it)
				vals.push_back(boost::lexical_cast<T>(*it));
			if (vals.size()) return true;
			return false;
		}

		template <class T> bool pNumeric(const std::string &in, T &val)
		{
			// Parse based on spaces
			typedef boost::tokenizer<boost::char_separator<char> >
				tokenizer;
			boost::char_separator<char> sep(" \t");
			tokenizer tcom(in,sep);
			auto it = tcom.begin();
			if (it == tcom.end()) return false;
			val = boost::lexical_cast<T>(*it);
			return true;
		}

		template <class T, class R> bool pMixed(const std::string &in, size_t numT,
			std::vector<T> &valsT, std::vector<R> &valsR)
		{
			valsT.clear();
			valsR.clear();
			// Separate based on strings
			// Parse based on spaces
			typedef boost::tokenizer<boost::char_separator<char> >
				tokenizer;
			boost::char_separator<char> sep(" \t");
			tokenizer tcom(in,sep);
			size_t np = 0;
			for (auto it=tcom.begin(); it != tcom.end(); ++it)
			{
				if (np < numT)
				{
					valsT.push_back(boost::lexical_cast<T>(*it));
				} else {
					valsR.push_back(boost::lexical_cast<R>(*it));
				}
				np++;
			}
			return true;
		}

		template <class T> bool pTuples(const std::string &in, const std::string &seps,
			size_t rank, std::vector< std::vector<T> > &vals)
		{
			vals.clear();
			typedef boost::tokenizer<boost::char_separator<char> >
				tokenizer;
			boost::char_separator<char> sep(seps.c_str());
			tokenizer tcom(in,sep);

			size_t r = 0;
			std::vector<T> vCurr;
			for (auto it=tcom.begin(); it != tcom.end(); ++it)
			{
				vCurr.push_back(boost::lexical_cast<T>(*it));
				r++;
				if (r == rank)
				{
					vals.push_back(vCurr);
					vCurr.clear();
				}
			}
			return true;
		}


	} // end parsers
} // end rtmath
