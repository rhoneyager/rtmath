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
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/shared_ptr.hpp>
#include "parids.h"
#include "../parsers.h"

namespace rtmath {
	namespace ddscat {

		class rotations;

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

				void setComment(const std::string &cmnt) { _comment = cmnt; }
				void writeComment(std::ostream &out)
				{
					if (_comment.size())
						out << _comment << std::endl;
				}
				//inline size_t line() const { return _line; }
				//void line(size_t nl) { _line = nl; }
				inline ParId id() const { return _id; }
				virtual bool versionValid(size_t ver) const;
				void endWriteWithEndl(bool val) { _endWriteWithEndl = val; }

				bool operator==(ddParLine &rhs);
				bool operator!=(ddParLine &rhs);
				bool operator<(ddParLine &rhs);

			protected:
				//size_t _line;
				ParId _id;
				bool _endWriteWithEndl;
				std::string _comment;
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
					: ddParLineSimpleBase<T>(id) {}
				~ddParLineSimple() {}
				virtual void write(std::ostream &out)
				{
					this->writeComment(out);
					std::string idstr;
					out << this->_val << " ";
					if (this->_endWriteWithEndl)
					{
						idString(this->_id,idstr);
						out << " = " << idstr << std::endl;
					}
				}
				virtual void read(const std::string &val)
				{
					std::istringstream strm(val);
					T trgt;
					strm >> trgt;
					this->set(trgt);
					//set( boost::lexical_cast<T>(val) );
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
					this->writeComment(out);
					std::string idstr;
					out << "\'" << _val << "\'";
					if (this->_endWriteWithEndl)
					{
						idString(this->_id,idstr);
						out << " = " << idstr << std::endl;
					}
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
					write(out,"");
				}
				virtual void write(std::ostream &out, const std::string &suffix)
				{
					this->writeComment(out);
					std::string idstr;
					for (auto it = _val.begin(); it != _val.end(); ++it)
						out << *it << " ";
					if (this->_endWriteWithEndl)
					{
						idString(this->_id,idstr);
						out << " = " << idstr << suffix << std::endl;
					}
				}
				virtual void read(const std::string &val)
				{
					_val.clear();
					// Parse based on spaces
					typedef boost::tokenizer<boost::char_separator<char> >
						tokenizer;
					boost::char_separator<char> sep(" \t(),");
					tokenizer tcom(val,sep);
					for (auto it=tcom.begin(); it != tcom.end(); ++it)
					{
						std::istringstream strm(*it);
						T trgt;
						strm >> trgt;
						_val.push_back(trgt);
						//set(trgt);
						//_val.push_back(boost::lexical_cast<T>(*it));
					}
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
					: ddParLineSimplePlural<T>(id) { _tuplesize = tuplesize; }
				virtual ~ddParTuples() {}
				virtual void write(std::ostream &out)
				{
					this->writeComment(out);
					std::string idstr;
					for (size_t i = 0; i < this->_val.size(); i++)
					{
						if (i % _tuplesize == 0)
							out << "(";
						out << this->_val[i];
						if ((i % _tuplesize) == (_tuplesize - 1))
						{
							out << ") ";
						} else {
							out << ",";
						}
					}
					if (this->_endWriteWithEndl)
					{
						idString(this->_id,idstr);
						out << " = " << idstr << std::endl;
					}
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
					this->writeComment(out);
					std::string idstr;
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
					if (this->_endWriteWithEndl)
					{
						idString(this->_id,idstr);
						out << " = " << idstr << std::endl;
					}
				}
				void setSep(size_t numT) const
				{
					_numT = numT;
				}
#pragma warning( push ) // Suppress warning. MSVC warning is because of how it branches,
#pragma warning( disable : 4244 4146 ) // even though that part of code is never reached
				template <class S>
				void get(size_t index, S &val) const
				{
					// Phrased like this to allow MSVC compile
					if (index >= _numT)
					{
						R pval;
						index -= _numT;
						_r[index].get(pval);
						val = boost::lexical_cast<S>(pval);
					} else {
						T pval;
						_t[index].get(pval);
						val = boost::lexical_cast<S>(pval);
					}
				}

				template <class S>
				void set(size_t index, const S &val)
				{
					if (index >= _numT)
					{
						index -= _numT;
						std::ostringstream srm;
						srm << val;
						std::istringstream irm(srm.str());
						R pval;
						irm >> pval;
						_r[index].set(pval);
					} else {
						std::ostringstream srm;
						srm << val;
						std::istringstream irm(srm.str());
						T pval;
						irm >> pval;
						_t[index].set(pval);
					}
				}
#pragma warning( pop ) 
			protected:
				mutable size_t _numT;
				std::vector<ddParLineSimple<T> > _t;
				std::vector<ddParLineSimple<R> > _r;
			};

			boost::shared_ptr<ddParLine> mapKeys(const std::string &key);
		}


#define accessorSimple(name,id,valtype) \
	valtype name() const \
		{ boost::shared_ptr<const ddParParsers::ddParLineSimple<valtype> > line; \
		boost::shared_ptr< const ddParParsers::ddParLine > linein; \
		valtype v; \
		getKey(id,linein); \
		line = boost::static_pointer_cast< const ddParParsers::ddParLineSimple<valtype> >(line); \
		line->get(v); \
		return v; } \
	void name(const valtype &v) \
		{ boost::shared_ptr< ddParParsers::ddParLineSimple<valtype> > line; \
		line->set(v); \
		insertKey(id,boost::static_pointer_cast< ddParParsers::ddParLine >(line)); \
		}

// Special case where a bool is stored in the file as an int
#define accessorSimpleBool(name,id) \
	bool name() const \
		{ boost::shared_ptr<const ddParParsers::ddParLineSimple<size_t> > line; \
		boost::shared_ptr< const ddParParsers::ddParLine > linein; \
		size_t v; \
		getKey(id,linein); \
		line = boost::static_pointer_cast< const ddParParsers::ddParLineSimple<size_t> >(line); \
		line->get(v); \
		return (v) ? true : false; } \
	void name(const bool &v) \
		{ boost::shared_ptr< ddParParsers::ddParLineSimple<size_t> > line; \
		size_t vi = (v) ? 1 : 0; \
		line->set(vi); \
		insertKey(id,boost::static_pointer_cast< ddParParsers::ddParLine >(line)); \
		}

#define accessorSimplePlural(name,id,valtype) \
	valtype name(size_t index) const \
		{ boost::shared_ptr<const ddParParsers::ddParLineSimplePlural<valtype> > line; \
		boost::shared_ptr< const ddParParsers::ddParLine > linein; \
		valtype v; \
		getKey(id,linein); \
		line = boost::static_pointer_cast< const ddParParsers::ddParLineSimplePlural<valtype> >(line); \
		line->get(index,v); \
		return v; } \
	void name(size_t index, const valtype &v) \
		{ boost::shared_ptr< ddParParsers::ddParLineSimplePlural<valtype> > line; \
		line->set(index,v); \
		insertKey(id,boost::static_pointer_cast< ddParParsers::ddParLine >(line)); \
		}

#define accessorString(getname,setname,id) \
	void getname(std::string &val) const \
		{ boost::shared_ptr<const ddParParsers::ddParLineSimple<std::string> > line; \
		boost::shared_ptr< const ddParParsers::ddParLine > linein; \
		std::string v; \
		getKey(id,linein); \
		line = boost::static_pointer_cast< const ddParParsers::ddParLineSimple<std::string> >(line); \
		line->get(v); \
		val = v; } \
	void setname(const std::string &v) \
		{ boost::shared_ptr< ddParParsers::ddParLineSimple<std::string> > line; \
		line->set(v); \
		insertKey(id,boost::static_pointer_cast< ddParParsers::ddParLine >(line)); \
		}

#define accessorStringBool(name,id,bfalse,btrue) \
	bool name() const \
		{ boost::shared_ptr<const ddParParsers::ddParLineSimple<std::string> > line; \
		boost::shared_ptr< const ddParParsers::ddParLine > linein; \
		std::string v; \
		getKey(id,linein); \
		line = boost::static_pointer_cast< const ddParParsers::ddParLineSimple<std::string> >(line); \
		line->get(v); \
		return (v == btrue) ? true : false; } \
	void name(bool v) \
		{ boost::shared_ptr<ddParParsers::ddParLineSimple<std::string> > line; \
		std::string vs = (v) ? btrue : bfalse ; \
		line->set(vs); \
		insertKey(id,boost::static_pointer_cast< ddParParsers::ddParLine >(line)); \
		}


		class ddPar
		{
		public:
			static ddPar*& defaultInstance();

			ddPar();
			ddPar(const std::string &filename);
			~ddPar();

			void readFile(const std::string &filename, bool overlay = false);
			void writeFile(const std::string &filename) const;
			void read(std::istream &stream, bool overlay = false);
			void write(std::ostream &stream) const;
			bool operator==(const ddPar &rhs) const;
			bool operator!=(const ddPar &rhs) const;
			inline size_t version() const { return _version; }
			inline void version(size_t nv) { _version = nv; }

			// Easy to use accessor functions
			accessorStringBool(doTorques,ddParParsers::CMTORQ,"NOTORQ","DOTORQ");
			accessorString(getSolnMeth,setSolnMeth,ddParParsers::CMDSOL);
			accessorString(getFFTsolver,setFFTsolver,ddParParsers::CMDFFT);
			accessorString(getCalpha,setCalpha,ddParParsers::CALPHA);
			accessorString(getBinning,setBinning,ddParParsers::CBINFLAG);
			accessorSimplePlural(Imem,ddParParsers::DIMENSION,size_t);

			accessorString(getShape,setShape,ddParParsers::CSHAPE);
			accessorSimplePlural(shpar,ddParParsers::SHAPEPARAMS,double);

			accessorSimpleBool(doNearField,ddParParsers::NRFLD);
			accessorSimplePlural(near,ddParParsers::FRACT_EXTENS,double);
			accessorSimple(maxTol,ddParParsers::TOL,double);
			accessorSimple(maxIter,ddParParsers::MXITER,size_t);
			accessorSimple(gamma,ddParParsers::GAMMA,double);
			accessorSimple(etasca,ddParParsers::ETASCA,double);

			void setWavelengths(double min, double max, size_t n, const std::string &spacing);
			void getWavelengths(double &min, double &max, size_t &n, std::string &spacing) const;

			accessorSimple(nambient,ddParParsers::NAMBIENT,double);

			void setAeff(double min, double max, size_t n, const std::string &spacing);
			void getAeff(double &min, double &max, size_t &n, std::string &spacing) const;

			accessorSimple(OrthPolState,ddParParsers::IORTH,size_t);
			accessorSimpleBool(writeSca,ddParParsers::IWRKSC);

			void getRots(rotations &rots) const;
			void setRots(const rotations &rots);

			// First IRAD
			// SIJ element number
			// SIJ indices

			accessorString(getCMDFRM,setCMDFRM,ddParParsers::CMDFRM);
			accessorSimple(numPlanes,ddParParsers::NPLANES,size_t);
			// Scattering planes
			
			// The older interface
			void insertKey(ddParParsers::ParId key, boost::shared_ptr<ddParParsers::ddParLine> ptr);
			void getKey(ddParParsers::ParId key, boost::shared_ptr<ddParParsers::ddParLine> &res);
			void getKey(ddParParsers::ParId key, boost::shared_ptr<const ddParParsers::ddParLine> &res) const;
			inline boost::shared_ptr<ddParParsers::ddParLine> getKey(ddParParsers::ParId key)
			{
				boost::shared_ptr<ddParParsers::ddParLine> res ;//= nullptr;
				getKey(key,res);
				return res;
			}
			void delKey(ddParParsers::ParId key);
			size_t size() const { return _parsedData.size(); }
			void getPlane(size_t key, boost::shared_ptr<ddParParsers::ddParLineSimplePlural<double> > &res);
			void getPlane(size_t key, boost::shared_ptr<const ddParParsers::ddParLineSimplePlural<double> > &res) const;
			inline boost::shared_ptr<ddParParsers::ddParLineSimplePlural<double> > getPlane(size_t key)
			{
				boost::shared_ptr<ddParParsers::ddParLineSimplePlural<double> > res ;//= nullptr;
				getPlane(key,res);
				return res;
			}
			void insertPlane(size_t key, boost::shared_ptr<ddParParsers::ddParLineSimplePlural<double>> &res);
			void delPlane(size_t key);
			friend class boost::serialization::access;
		private:
			void _init();
			size_t _version;
			void _populateDefaults(bool overwrite = false, const std::string &src = "") const;

			mutable std::map<ddParParsers::ParId, 
				boost::shared_ptr<ddParParsers::ddParLine> > 
				_parsedData;

			mutable std::map<size_t,
				boost::shared_ptr<ddParParsers::ddParLineSimplePlural<double> > > 
				_scaPlanes;

			std::map<size_t, std::string> _comments;
			//std::string _savedata;

			template<class Archive>
			void save(Archive & ar, const unsigned int version) const
			{
				//ar & BOOST_SERIALIZATION_NVP(exportLoc);
				//ar & boost::serialization::make_nvp("ddPar_Raw", _parsedData);
				//ar & boost::serialization::make_nvp("Scattering_Planes", _scaPlanes);
				std::ostringstream out;
				write(out);
				std::string _savedata;
				_savedata = out.str();
				ar & boost::serialization::make_nvp("Par_File", _savedata);
			}
			template<class Archive>
			void load(Archive & ar, const unsigned int version)
			{
				std::string _savedata;
				ar & boost::serialization::make_nvp("Par_File", _savedata);
				std::istringstream in(_savedata);
				read(in);
			}
			BOOST_SERIALIZATION_SPLIT_MEMBER()
		};

#undef accessorSimple
#undef accessorSimpleBool
#undef accessorSimplePlural
#undef accessorString
#undef accessorStringBool

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

