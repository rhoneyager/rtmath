#pragma once
/** Contains common templates used in various functions. **/

#include <set>
#include <vector>
#include <string>
#include <boost/lexical_cast.hpp>

#include "defs.h"
#include <Ryan_Debug/splitSet.h>

namespace rtmath {
	template <class T>
	class paramSet;
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
			Ryan_Debug::splitSet::splitSet<T>(_shorthand, _expanded, _aliases);
		}
	};


	
	/// Allows for easy ordering of objects within sets
	template <typename T>
	struct sharedComparator
	{
		bool operator()(const T &lhs, const T &rhs) const
		{
			bool res = (lhs)->operator<(*rhs);
			return res;
		}
	};
}
