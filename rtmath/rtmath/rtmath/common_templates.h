#pragma once
/* Contains common templates used in various functions.
* Such as a null_deleter used to elude shared_ptr 
* difficulties with *this.
*/

#include <sstream>
#include <set>
#include <vector>
#include <string>
#include <boost/tuple/tuple.hpp> 

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
		typedef std::map<std::string, std::string> aliasmap;
		paramSet(const aliasmap *aliases = nullptr) 
			{ _aliases = aliases; }
		paramSet(const std::string &src, const aliasmap *aliases = nullptr) 
			{ _aliases = aliases; set(src); }
		paramSet(const T &src)
		{
			_aliases = nullptr;
			set(boost::lexical_cast<std::string>(src));
			/*
			std::ostringstream out;
			out << src;
			std::string t = out.str();
			set(t);
			*/
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
		friend struct std::less<rtmath::paramSet<T> >;
		friend class boost::serialization::access;
	};

	// Supporting code to allow boost unordered maps
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

} // end namespace std




