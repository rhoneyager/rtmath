#include <fstream>
#include <iostream>

#include <boost/algorithm/string/trim.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
//#include <boost/tokenizer.hpp>

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/spirit/include/qi_omit.hpp>
#include <boost/spirit/include/qi_repeat.hpp>

#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/iostreams/filtering_stream.hpp>

#include "parser.h"
#include "observation.h"

namespace {
	namespace qi = boost::spirit::qi;
	namespace ascii = boost::spirit::ascii;
	namespace phoenix = boost::phoenix;

	/// Parses space-separated numbers
	template <typename Iterator>
	bool parse_numbers_space(Iterator first, Iterator last, std::vector<float>& v)
	{
		using qi::float_;
		using qi::phrase_parse;
		using qi::_1;
		using ascii::space;
		using phoenix::push_back;

		bool r = phrase_parse(first, last,

			//  Begin grammar
			(
			*float_
			)
			,
			//  End grammar

			space, v);

		if (first != last) // fail if we did not get a full match
			return false;
		return r;
	}
}

void parse_file(const std::string &filename, std::vector<observation> &obs)
{
	// For better performance, the input files will be memory-mapped
	using namespace boost::interprocess;
	using namespace boost::filesystem;
	size_t fsize = (size_t)file_size(path(filename)); // bytes

	file_mapping m_file(
		filename.c_str(),
		read_only
		);

	mapped_region region(
		m_file,
		read_only,
		0,
		fsize);

	void* start = region.get_address();
	const char* a = (char*)start;

	std::vector<float> vals;
	vals.reserve(6000000);
	parse_numbers_space(a, a + fsize, vals);
	std::cerr << "Vals #: " << vals.size() << "\t"
		<< (int)(vals.size()) / 10 << "\t" << (int)(vals.size()) % 10 << std::endl;

	for (size_t i = 0; i < vals.size(); i += 10)
	{
		// Constructor makes ingest easy
		observation o(vals.data() + i);
		obs.push_back(std::move(o));
	}

	std::cerr << "Read " << vals.size() / 10 << " observations." << std::endl;

}
