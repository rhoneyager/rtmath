#pragma once

#include <string>
#include <vector>

inline void splitString(const std::string &source, char delim, std::vector<std::string> &res)
{
	// Take the string, find all occurances of the delimeter, and split the string
	// into substrings.
	// This includes empty substrings (i.e. "1,,23s")
	res.clear();
	size_t start = 0;
	size_t end = 0;
	size_t span;
	size_t stop = source.size();
	while (start < stop)
	{
		// Find the next item to be split
		end = source.find_first_of(delim,start);
		span = end - start;
		if (end != start)
		{
			std::string sub = source.substr(start,span);
			res.push_back(sub);
		} else {
			// The end position is the same as the start position
			res.push_back("");
		}
		if (end == std::string::npos) break;
		start = end + 1;
	}
}


