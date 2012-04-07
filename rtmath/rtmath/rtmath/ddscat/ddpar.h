#pragma once
/* This header contains the functions for parsing ddscat.par files. It is used to both 
 * extract information necessary for shapefile formation, and is used when upgrading 
 * ddscat versions. It also provides writing functionality for the files.
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <complex>
#include <string>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include "parids.h"

namespace rtmath {
	namespace ddscat {

		namespace ddParParsers
		{
			class ddParLine;
		}

		class ddPar
		{
		public:
			ddPar();
			ddPar(const std::string &filename);
			~ddPar();

			void loadFile(const std::string &filename);
			void saveFile(const std::string &filename) const;
			void load(std::istream &stream);
			void parseMap();
			inline size_t version() const { return _version; }
			inline void version(size_t nv) { _version = nv; }
		private:
			void _init();
			size_t _version;
			void _populateDefaults() const;
			mutable std::map<std::string, std::string> _keys;

			mutable std::map<ddParParsers::ParId, 
				std::shared_ptr<ddParParsers::ddParLine> > 
				_parsedData;

			std::map<size_t, std::string> _comments;
		};

		namespace ddParParsers
		{
			void idString(ParId id, std::string &key);

			// A vert abstract base class
			class ddParLine
			{
			public:
				friend struct std::less<rtmath::ddscat::ddParParsers::ddParLine>;
				ddParLine(ParId id = UNKNOWN) { _id = id; }
				virtual ~ddParLine() {}
				virtual void read(std::istream &in) = 0;
				virtual void read(const std::string &val) = 0;
				virtual void write(std::ostream &out) = 0;

				//inline size_t line() const { return _line; }
				//void line(size_t nl) { _line = nl; }
				inline ParId id() const { return _id; }
				virtual bool versionValid(size_t ver) const;
			protected:
				//size_t _line;
				ParId _id;
			};

			// Added this as a level of abstraction to prevent code duplication
			template <class T>
			class ddParLineSimpleBase : public ddParLine
			{
			public:
				ddParLineSimpleBase(ParId id = UNKNOWN) 
					: ddParLine(id) {}
				virtual ~ddParLineSimpleBase() {}
				virtual void read(std::istream &in)
				{
					// No key / val separation here...
					std::string lin;
					std::getline(in,lin);
					read(lin);
				}
				virtual void read(const std::string &val) = 0;
				virtual void write(std::ostream &out) = 0;

				void get(T &val) const { val = _val; }
				void set(const T &val) { _val = val; }
			protected:
				T _val;
			};

			// std::string is specialized later
			template <class T>
			class ddParLineSimple : public ddParLineSimpleBase<T>
			{
			public:
				ddParLineSimple(ParId id = UNKNOWN) 
					: ddParLineSimpleBase(id) {}
				~ddParLineSimple() {}
				virtual void write(std::ostream &out)
				{
					std::string idstr;
					idString(_id,idstr);
					out << _val << "   = " << idstr << std::endl;
				}
				virtual void read(const std::string &val)
				{
					set( boost::lexical_cast<T>(val) );
				}
			};

			// Put parser def here so the subsequent class can load it
			void pString(const std::string &in, std::string &out);

			template <>
			class ddParLineSimple <std::string> : public ddParLineSimpleBase<std::string>
			{
			public:
				ddParLineSimple(ParId id = UNKNOWN) 
					: ddParLineSimpleBase(id) {}
				~ddParLineSimple() {}
				virtual void write(std::ostream &out)
				{
					std::string idstr;
					idString(_id,idstr);
					out << "\'" << _val << "\'   = " << idstr << std::endl;
				}
				virtual void read(const std::string &val)
				{
					std::string pstr;
					pString(val,pstr);
					set(pstr);
				}
			};

			
			// The rest of the parsers go here
			// TODO:  CONVERT THESE LIKE WITH THE SIMPLE CASES!!!

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
		}

	}
}

namespace std
{
	template <> struct less<rtmath::ddscat::ddParParsers::ddParLine >
	{
		bool operator() (const rtmath::ddscat::ddParParsers::ddParLine &lhs, const rtmath::ddscat::ddParParsers::ddParLine &rhs) const
		{
			// Only do ordering based on id
			if (lhs._id != rhs._id) return lhs._id < rhs._id;
			return false;
		}
	};
}

