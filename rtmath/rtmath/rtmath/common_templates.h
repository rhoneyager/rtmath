#pragma once
/* Contains common templates used in various functions.
 * Such as a null_deleter used to elude shared_ptr 
 * difficulties with *this.
 */

#include <set>
#include <vector>
#include <string>
#include <boost/tuple/tuple.hpp> 
#include <boost/serialization/nvp.hpp> 
#include <boost/preprocessor/repetition.hpp> // used for boost tuple serialization
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/string.hpp> 
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include "defs.h"
#include "command.h"
#include "Public_Domain/MurmurHash3.h"

struct null_deleter
{
	template <class T> void operator()(T *) {}
};


namespace rtmath
{
	// Template functions that tie into command.h
	// These can hold a set of values and convert to/from a string using the command.h notation
	// Conveniently, this can be extended to also give units or variable type with std::pair
	// or the larger boost library equivalent

	class hashable
	{
	public:
		hashable() {}
		virtual ~hashable() {}
		inline HASH_t hash() const
		{
			HASH_t res;
			HASH(this, sizeof(*this), HASHSEED, &res);
			return res;
		}
	};

	template <class T>
	class paramSet : public hashable
	{
	public:
		paramSet() {}
		~paramSet() {}
		void set(const std::string &str)
		{
			_shorthand = str;
			_expand();
		}
		void getShort(std::string &str) const
		{
			str = _shorthand;
		}
		void getLong(std::vector<T> &expanded) const
		{
			expanded = _expanded;
		}
		void getLong(size_t index, T &expanded) const
		{
			expanded = _expanded[index];
		}
		typename std::vector<T>::const_iterator begin() const
		{
			return _expanded.begin();
		}
		typename std::vector<T>::const_iterator end() const
		{
			return _expanded.end();
		}
		bool operator< (const paramSet<T> &rhs) const
		{
			return _shorthand < rhs._shorthand;
		}
	private:
		std::vector<T> _expanded;
		std::string _shorthand;
		void _expand()
		{
			rtmath::config::splitSet<T>(_shorthand, _expanded);
		}
		friend struct std::less<rtmath::paramSet<T> >;
		friend class boost::serialization::access;
	public:
		template<class Archive>
			void serialize(Archive & ar, const unsigned int version)
			{
				ar & BOOST_SERIALIZATION_NVP(_shorthand);
			}
	};

	// Supporting code to allow boost unordered map
	template <class T> std::size_t hash_value(rtmath::paramSet<T> const& x)
	{
		return (size_t) x.hash();
	}
}



namespace std {
	template <typename T> struct hash<rtmath::paramSet<T> >
	{
		size_t operator()(const rtmath::paramSet<T> & x) const
		{
			// Really need to cast for the unordered map to work
			return (size_t) x.hash();
		}
	};


	template <typename T> struct less<rtmath::paramSet<T> >
	{
		bool operator() (const rtmath::paramSet<T> &lhs, const rtmath::paramSet<T> &rhs) const
		{
			if (lhs._shorthand != rhs._shorthand) return lhs._shorthand < rhs._shorthand;

			return false;
		}
	};
} // end namespace std



// boost tuple serialization - from http://uint32t.blogspot.com/2008/03/update-serializing-boosttuple-using.html
namespace boost { namespace serialization {

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

}}


