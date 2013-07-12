#pragma once
/* Contains common templates used in various functions.
*/

//#include <sstream>
#include <set>
#include <vector>
#include <string>
//#include <boost/tuple/tuple.hpp> 
#include <boost/lexical_cast.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/access.hpp>

#include "defs.h"
#include "splitSet.h"
#include "Public_Domain/MurmurHash3.h"

// Forward declaration for boost::serialization below
namespace rtmath {
	template <class T>
	class paramSet;
}

namespace boost
{
	namespace serialization
	{
		/// Definition to serialize all paramSet objects.
		/// \note Need to use the external definition because of a MSVC 2012 bug.
		/// \todo Check for bug resolution in MSVC 2013.
		template <class Archive, class T>
		void serialize(Archive & ar, rtmath::paramSet<T> & g, const unsigned int version);
	}
/// \todo Fix boost tuple serialization to work with LLVM/CLANG.
		// boost tuple serialization - from http://uint32t.blogspot.com/2008/03/update-serializing-boosttuple-using.html
		// Breaks in CLANG!
/*
#define GENERATE_ELEMENT_SERIALIZE(z,which,unused) \
	ar & boost::serialization::make_nvp("element",t.get< which >());

#define GENERATE_TUPLE_SERIALIZE(z,nargs,unused)                        \
	template< typename Archive, BOOST_PP_ENUM_PARAMS(nargs,typename T) > \
	void serialize(Archive & ar,                                        \
	boost::tuple< BOOST_PP_ENUM_PARAMS(nargs,T) > & t,   \
	const unsigned int version)                          \
		{                                                                   \
		BOOST_PP_REPEAT_FROM_TO(0,nargs,GENERATE_ELEMENT_SERIALIZE,~);    \
		}


		BOOST_PP_REPEAT_FROM_TO(1,6,GENERATE_TUPLE_SERIALIZE,~);
*/
}

namespace rtmath
{
	// Template functions that tie into command.h
	// These can hold a set of values and convert to/from a string using the command.h notation
	// Conveniently, this can be extended to also give units or variable type with std::pair
	// or the larger boost library equivalent

	template <class T>
	class paramSet
	{
		friend class ::boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version);
		template<class Archive, class T> friend
		void ::boost::serialization::serialize(Archive & ar, rtmath::paramSet<T> & g, const unsigned int version);
	public:
		typedef std::map<std::string, std::string> aliasmap;
		paramSet(const aliasmap *aliases = nullptr) 
			{ _aliases = aliases; }
		paramSet(const std::string &src, const aliasmap *aliases = nullptr) 
			{ _aliases = aliases; set(src); }
		paramSet(const T &src)
		{
			_aliases = nullptr;
			set(boost::lexical_cast<std::string>(src));
		}
		~paramSet() {}
		size_t size() const
		{
			return _expanded.size();
		}
		void set(const std::string &str)
		{
			_shorthand = str;
			_expand();
		}
		void getShort(std::string &str) const
		{
			str = _shorthand;
		}
		void getLong(std::set<T> &expanded) const
		{
			expanded = _expanded;
		}
		void getLong(size_t index, T &expanded) const
		{
			expanded = _expanded[index];
		}
		typedef typename std::set<T>::const_iterator const_iterator;
		typedef typename std::set<T>::const_reverse_iterator const_reverse_iterator;
		const_iterator begin() const
		{
			return _expanded.begin();
		}
		const_iterator end() const
		{
			return _expanded.end();
		}
		const_reverse_iterator rbegin() const
		{
			return _expanded.rbegin();
		}
		const_reverse_iterator rend() const
		{
			return _expanded.rend();
		}
		bool operator< (const paramSet<T> &rhs) const
		{
			return _shorthand < rhs._shorthand;
		}
		bool operator== (const paramSet<T> &rhs) const
		{
			if (_shorthand == rhs._shorthand) return true;
			return false;
		}
		inline bool operator!= (const paramSet<T> &rhs) const
		{
			return !(operator==(rhs));
		}
	private:
		std::set<T> _expanded;
		std::string _shorthand;
		const std::map<std::string, std::string> *_aliases;
		void _expand()
		{
			rtmath::config::splitSet<T>(_shorthand, _expanded, _aliases);
		}
	};

}

BOOST_CLASS_EXPORT_KEY(rtmath::paramSet<double>);
BOOST_CLASS_EXPORT_KEY(rtmath::paramSet<float>);
BOOST_CLASS_EXPORT_KEY(rtmath::paramSet<int>);
BOOST_CLASS_EXPORT_KEY(rtmath::paramSet<size_t>);
BOOST_CLASS_EXPORT_KEY(rtmath::paramSet<std::string>);
