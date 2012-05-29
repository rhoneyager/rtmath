#pragma once
/* Contains common templates used in various functions.
 * Such as a null_deleter used to elude shared_ptr 
 * difficulties with *this.
 */

#include <set>
#include <vector>
#include <string>
#include "command.h"

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

	template <class T>
	class paramSet
	{
	public:
		paramSet();
		~paramSet();
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
	private:
		std::vector<T> _expanded;
		std::string _shorthand;
		void _expand()
		{
			rtmath::config::splitSet<T>(_shorthand, expanded);
		}
	};

}

