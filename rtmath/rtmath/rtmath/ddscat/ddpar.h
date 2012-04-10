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
#include "../parsers.h"

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
			inline size_t version() const { return _version; }
			inline void version(size_t nv) { _version = nv; }
			void insertKey(ddParParsers::ParId key, std::shared_ptr<ddParParsers::ddParLine> &ptr);
			std::shared_ptr<ddParParsers::ddParLine> getKey(ddParParsers::ParId key);
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
				ddParLine(ParId id = UNKNOWN) { _id = id; _endWriteWithEndl = true; }
				virtual ~ddParLine() {}
				// Read does NOT split keys and vals!!!!!
				virtual void read(std::istream &in)
				{
					// No key / val separation here...
					std::string lin;
					std::getline(in,lin);
					read(lin);
				}
				virtual void read(const std::string &val) = 0;
				virtual void write(std::ostream &out) = 0;

				//inline size_t line() const { return _line; }
				//void line(size_t nl) { _line = nl; }
				inline ParId id() const { return _id; }
				virtual bool versionValid(size_t ver) const;
				void endWriteWithEndl(bool val) { _endWriteWithEndl = val; }
			protected:
				//size_t _line;
				ParId _id;
				bool _endWriteWithEndl;
			};

			// Added this as a level of abstraction to prevent code duplication
			template <class T>
			class ddParLineSimpleBase : public ddParLine
			{
			public:
				ddParLineSimpleBase(ParId id = UNKNOWN) 
					: ddParLine(id) {}
				virtual ~ddParLineSimpleBase() {}
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
					out << _val << " ";
					if (_endWriteWithEndl)
						out << " = " << idstr << std::endl;
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
					out << "\'" << _val << "\'";
					if (_endWriteWithEndl)
						out << " = " << idstr << std::endl;
				}
				virtual void read(const std::string &val)
				{
					std::string pstr;
					pString(val,pstr);
					set(pstr);
				}
			};

			template <class T>
			class ddParLineSimplePlural : public ddParLine
			{
			public:
				ddParLineSimplePlural(ParId id = UNKNOWN) 
					: ddParLine(id) {}
				virtual ~ddParLineSimplePlural() {}
				virtual void write(std::ostream &out)
				{
					std::string idstr;
					idString(_id,idstr);
					for (auto it = _val.begin(); it != _val.end(); ++it)
						out << *it << " ";
					if (_endWriteWithEndl)
						out << " = " << idstr << std::endl;
				}
				virtual void read(const std::string &val)
				{
					_val.clear();
					// Parse based on spaces
					typedef boost::tokenizer<boost::char_separator<char> >
						tokenizer;
					boost::char_separator<char> sep(" \t()");
					tokenizer tcom(val,sep);
					for (auto it=tcom.begin(); it != tcom.end(); ++it)
						_val.push_back(boost::lexical_cast<T>(*it));
				}

				void get(size_t index, T &val) const { val = _val.at(index); }
				void set(size_t index, const T &val) { _val[index] = val; }
				void get(std::vector<T> &val) const { val = _val; }
				void set(const std::vector<T> &val) { _val = val; }
			protected:
				std::vector<T> _val;
			};
			
			// This is a special case for paired numbers.
			template <class T>
			class ddParTuples : public ddParLineSimplePlural<T>
			{
			public:
				ddParTuples(size_t tuplesize, ParId id = UNKNOWN) 
					: ddParLineSimplePlural(id) { _tuplesize = tuplesize; }
				virtual ~ddParTuples() {}
				virtual void write(std::ostream &out)
				{
					std::string idstr;
					idString(_id,idstr);
					for (size_t i = 0; i < _val.size(); i++)
					{
						if (i % _tuplesize == 0)
							out << "(";
						out << _val[i];
						if ((i % _tuplesize) == (_tuplesize - 1))
						{
							out << ") ";
						} else {
							out << ",";
						}
					}
					if (_endWriteWithEndl)
						out << " = " << idstr << std::endl;
				}
			protected:
				size_t _tuplesize;
			};

			template <class T, class R> 
			class ddParLineMixed : public ddParLine
			{
			public:
				ddParLineMixed(size_t numT, ParId id = UNKNOWN) 
					: ddParLine(id) { _numT = numT; }
				virtual ~ddParLineMixed() {}
				virtual void read(const std::string &val)
				{
					// Separate based on spaces
					_t.clear();
					_r.clear();
					// Parse based on spaces
					typedef boost::tokenizer<boost::char_separator<char> >
						tokenizer;
					boost::char_separator<char> sep(" \t()");
					tokenizer tcom(val,sep);
					size_t num = 0;
					for (auto it=tcom.begin(); it != tcom.end(); ++it)
					{
						//val.push_back(*it);
						// Select where to insert based on position
						if (num < _numT)
						{
							ddParLineSimple<T> in;
							in.read(*it);
							_t.push_back(in);
						} else {
							ddParLineSimple<R> in;
							in.read(*it);
							_r.push_back(in);
						}
						num++;
					}
				}
				virtual void write(std::ostream &out)
				{
					std::string idstr;
					idString(_id,idstr);
					// Each separate member is used to write
					// Suppress the endline emitted by the members
					for (auto it = _t.begin(); it != _t.end(); ++it)
					{
						it->endWriteWithEndl(false);
						it->write(out);
					}
					for (auto it = _r.begin(); it != _r.end(); ++it)
					{
						it->endWriteWithEndl(false);
						it->write(out);
					}
					if (_endWriteWithEndl)
						out << " = " << idstr << std::endl;
				}
			protected:
				size_t _numT;
				std::vector<ddParLineSimple<T> > _t;
				std::vector<ddParLineSimple<R> > _r;
			};

			std::shared_ptr<ddParLine> mapKeys(const std::string &key);
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

