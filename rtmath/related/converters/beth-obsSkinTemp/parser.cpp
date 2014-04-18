#include <fstream>
#include <iostream>
#include <mutex>
#include <queue>
#include <sstream>
#include <thread>

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

#include <Ryan_Debug/debug.h>
#include <Ryan_Serialization/serialization.h>

#include "parser.h"
#include "observation.h"

namespace {
	namespace qi = boost::spirit::qi;
	namespace ascii = boost::spirit::ascii;
	namespace phoenix = boost::phoenix;

	/// Parses space-separated numbers
	template <typename Iterator>
	bool parse_numbers_space(Iterator first, Iterator last, std::vector<double>& v)
	{
		using qi::double_;
		using qi::phrase_parse;
		using qi::_1;
		using ascii::space;
		using phoenix::push_back;

		bool r = phrase_parse(first, last,

			//  Begin grammar
			(
			*double_
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
	using namespace Ryan_Serialization;
	using std::string;

	std::string cmeth, fname;
	detect_compressed(filename, cmeth, fname);

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

	void* regionStart = region.get_address();
	const char* a = (char*) regionStart;

	
	string s(a, fsize);
	if (cmeth.size())
	{
		std::istringstream ss(s);

		boost::iostreams::filtering_istream sin;
		// sin can contain either compressed or uncompressed input at this point.
		if (cmeth.size())
			prep_decompression(cmeth, sin);
		sin.push(ss);

		string suncompressed;
		suncompressed.reserve(1024 * 1024 * 150);
		std::ostringstream so;
		boost::iostreams::copy(sin, so);
		suncompressed = so.str();
		s.swap(suncompressed);
		//std::istringstream ss_unc(suncompressed);
	}
	
	// Fast read scheme reads the file in chunks.
	// Each chunk reads to the first error, EOF, or a max of 5000 lines.

	std::mutex m_pool;
	std::vector<std::thread> pool;
	std::queue<std::pair<char*, char*> > q;
	// Could do with iterators. Might yield nicer code.
	const char* p = s.data(); //suncompressed.data();
	const char* const pend = s.data() + s.size(); //suncompressed.data() + suncompressed.size();

	size_t nErrors = 0;

	
	const size_t nMatches = 3;
	const char matches[nMatches] = {'N', 'a', '*'};
	const size_t nNums = 12;
	const char ints[nNums] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '-', '.'};

	auto process_chunk = [&](char * const start, char * const stop)
	{
		try {
		// Replace all of the bad lines with spaces
		size_t nErrorsLocal = 0;

		char *p = start;
		while (p < stop)
		{
		
			auto it = std::find_first_of(p+1, stop, matches, matches+nMatches);
			if (it != stop)
			{
				auto next = std::find(it, stop, '\n');
				const char lend[] = {'\n'};
				auto prev = std::find_end(p, it, lend, lend+1);
				if (prev == it) prev = p;
				if (next >= stop) next = stop-1;
				prev = std::find_first_of(prev, prev+8, ints, ints+nNums); // Advance past \r\n
				std::fill(prev, next, ' ');
				nErrorsLocal++;
				p = next + 1;
			} else {
				break;
			}
		}

		
		/*{
			std::ofstream out("debug.dat");
			out << std::string(start,stop);
			out << std::endl;
		}*/
		

		// Count the lines
		size_t nLines = std::count(start, stop, '\n');

		// Parse the lines
		std::vector<double> vals;
		vals.reserve(nLines*10);

		if (!parse_numbers_space(start, stop, vals))
			throw std::string("Error parsing chunk - parser error");
		if (vals.size() % 10 )
				throw std::string("Error parsing chunk - uneven alignment");

		std::vector<observation> temp;
		temp.reserve(nLines);
		for (size_t i = 0; i < vals.size(); i += 10)
		{
			// Constructor makes ingest easy
			observation o(vals.data() + i);
			temp.push_back(std::move(o)); // Place in the designated observation slot
		}

		std::lock_guard<std::mutex> lock(m_pool);
		obs.insert(obs.end(), temp.begin(), temp.end());
		nErrors += nErrorsLocal;
		} catch (std::string &s)
		{
			std::cerr << s << std::endl;
		}
	};

	auto process_chunk_queue = [&]()
	{
		for (;;)
		{
			std::pair<char*, char*> bnds;
			{
				std::lock_guard<std::mutex> lock(m_pool);
				if (!q.size()) return;
				bnds = q.front();
				q.pop();
			}
			process_chunk(bnds.first, bnds.second);
		}
	};
	
	// For every 5000 lines (or eof), spawn a thread to process the data
	//q.push(std::pair<char*, char*>(const_cast<char*>(p), const_cast<char*>(pend) )); // Space inserter debugging
	while (p < pend) // Regular execution loop
	{
		const char* stop = p;
		// Take a guess at where the end of line is around (assuming fixed 75 column lines)
		auto it = stop + (75*5000);
		if (it < pend)
		{
			it = std::find(it+1, pend, '\n');
			it = std::find_first_of(it, it+8, ints, ints+nNums); // Advance past \r\n to first number
		}
		if (it > pend) it = pend;

		stop = it;

		//for (size_t i=0; i<5000; ++i)
		//{
			//auto it = std::find(stop+1, pend, '\n');
			//stop = it;
			//if (it >= pend) break;
		//}
		q.push(std::pair<char*, char*>(const_cast<char*>(p), const_cast<char*>(stop) ));
		//int span = stop - p;
		p = stop + 1;
	}
	
	
	size_t numThreads = Ryan_Debug::getConcurrentThreadsSupported();
	for (size_t i = 0; i<numThreads; i++)
	{
		std::thread t(process_chunk_queue);
		pool.push_back(std::move(t));
	}

	for (size_t i = 0; i<pool.size(); i++)
	{
		pool[i].join();
	}

	
	std::cerr << "Total obs: " << obs.size() << std::endl;
	//std::cerr << "Read " << i << " lines, with " << nErrors << " errors." << std::endl;

}
