#include <fstream>
#include <iostream>

#include <boost/algorithm/string/trim.hpp>
#include <boost/lexical_cast.hpp>
//#include <boost/tokenizer.hpp>

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/spirit/include/qi_omit.hpp>
#include <boost/spirit/include/qi_repeat.hpp>


#include "parser.h"
#include "station.h"

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

void parse_file(int month, int year, const std::string &filename, std::map<int,station> &stations)
{
	std::ifstream in(filename.c_str());
	// Iterate over all lines in the file. 
	// Only process lines with the third character being a number.
	// This conveniently ignores all other fields.
	while (in.good())
	{
		std::string lin;
		std::getline(in, lin);
		try {
			//boost::trim(lin);
			if (!lin.size()) continue;
			if (lin.size() < 15) continue;
			if (!std::isdigit(lin.at(1))) continue;

			using std::string;
			string slat = lin.substr(0,6);
			string slon = lin.substr(7,7);
			string scoop = lin.substr(15,6);
			string sstnid = lin.substr(22,8);
			string state = lin.substr(31,2);
			string city = lin.substr(34,30);
			string county = lin.substr(66,25);
			string selev = lin.substr(93,6);
			boost::trim(slat);
			boost::trim(slon);
			boost::trim(scoop);
			boost::trim(sstnid);
			boost::trim(state);
			boost::trim(city);
			boost::trim(county);
			boost::trim(selev);

			// After this are the snowfall rates for the given dates in the month
			std::vector<float> vals;
			vals.reserve(31);
			parse_numbers_space(lin.begin()+100, lin.end(), vals);

			using boost::lexical_cast;
			int COOP = lexical_cast<int>(scoop);
			float lat = lexical_cast<float>(slat);
			float lon = lexical_cast<float>(slon);
			float elev = lexical_cast<float>(selev);
			
			// Add to / construct station information
			if (!stations.count(COOP))
			{
				station newstation(COOP, lat, lon, elev, sstnid, state, city, county);
				stations[COOP] = std::move(newstation);
			}

			for (size_t i=0; i<vals.size(); ++i)
			{
				using namespace boost::posix_time;
				using namespace boost::gregorian;
				date d(year, month, (unsigned short) (i+1) );
				//s->startTime = ptime(b, seconds(static_cast<long>(base_time(0, 0))));
				stations[COOP].addObs(d, vals[i]);
			}

		} catch (std::exception &e) {
			std::cerr << "Cannot parse: " << e.what() << std::endl
				<< "Line: " << lin << std::endl
				<< "Filename: " << filename << std::endl;
		}
	}
}
