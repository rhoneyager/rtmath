// This is my version of Dr. Liu's rpline program.
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <boost/filesystem.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <Ryan_Debug/debug.h>
#include "../../rtmath/rtmath/error/debug.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	try {
		cerr << "rtmath-rpline" << endl;

		namespace po = boost::program_options;

		po::options_description desc("Allowed options"), cmdline("Command-line options"),
		config("Config options"), hidden("Hidden options"), oall("all options");
		//rtmath::ddscat::shapeFileStats::add_options(cmdline, config, hidden);
		rtmath::debug::add_options(cmdline, config, hidden);

		/// \todo Extend program to make multiple substitutions at the same time.
		cmdline.add_options()
			("help,h", "produce help message")
			("output,o", po::value<string>(), 
			"Specify output file")
			("input,i", po::value<string >(), 
			"Specify input file.")
			
			("replace,r", po::value<string>(),
			"The text to replace with.")
			("number,n", po::value<size_t>(),
			"Replace the nth delimited region")

			("delimiter,d", po::value<string>(),
			"Set the delimiter for input and output. If not specified, defaults "
			"to a newline.")
			
			;

		po::positional_options_description p;
		//p.add("inputs",-1);

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);


		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);

		auto doHelp = [=](const std::string& message) {
			if (message.size())
				std::cerr << message << endl << endl;
			std::cerr << desc << "\n";
			exit(2);
		};
		if (vm.count("help") || argc == 1) doHelp("");

		if (!vm.count("input")) doHelp("Must specify an input file.");
		if (!vm.count("output")) doHelp("Must specify an output file.");
		if (!vm.count("replace")) doHelp("Must specify replacement text.");
		if (!vm.count("number")) doHelp("Must specify the text region to replace.");


		string infile = vm["input"].as<string>();
		string outfile = vm["output"].as<string>();
		string reptext = vm["replace"].as<string>();
		size_t sepnum = vm["number"].as<size_t>();
		if (!sepnum) doHelp("Parameter number must be greater than zero.");
		sepnum--;
		string outSep = "\n";
		if (vm.count("delimiter"))
			outSep = vm["delimiter"].as<string>();


		// Map the file into memory
		/// \todo Change so that only parts of a large file are mapped at once.
		using namespace boost::filesystem;
		using namespace boost::interprocess;
		if (!exists(path(infile))) throw debug::xMissingFile(infile.c_str());
		
		size_t fsize = (size_t) file_size(path(infile)); // bytes
		file_mapping m_in(
			infile.c_str(),
			read_only
			);

		mapped_region region (
			m_in,
			read_only,
			0,
			fsize);
		void* vstart = region.get_address();

		// start is the beginning of the file
		const char* start = (char*) vstart;
		// end is the end of the file
		const char* end = start + fsize;
		// repstart is the start of the replacement region
		const char *repstart = start, *repend = start;
		for (size_t i=0; i<sepnum; ++i)
		{
			repstart = std::find_first_of(repstart, end, outSep.begin(), outSep.end());
			if (repstart == end)
				throw std::string("End of file before start of the replacement region was found.");
			repstart++;
		}
		// repend is the end of the replacement region
		repend = std::find_first_of(repstart+1, end, outSep.begin(), outSep.end());

		
		// Calculate the size of the resultant file
		size_t resSize = fsize + reptext.size() - (repend - repstart);

		/*
		{
			std::filebuf fbuf;
			fbuf.open(outfile.c_str(), std::ios_base::in | std::ios_base::out
				| std::ios_base::trunc | std::ios_base::binary);
			//Set the size
			fbuf.pubseekoff(resSize-1, std::ios_base::beg);
			fbuf.sputc(0);
		}
		*/
		ofstream out(outfile.c_str(), std::ios_base::out | std::ios_base::binary);
		out.write(start, repstart - start);
		out.write(reptext.data(), reptext.size());
		out.write(repend, end - repend);
		
		/*
		file_mapping m_out(
			outfile.c_str(),
			read_write
			);

		mapped_region region_out (
			m_out,
			read_write,
			0,
			fsize);
		void* vostart = region_out.get_address();

		char* ostart = (char*) vostart;
		std::copy(start, repstart, ostart);
		//std::copy(reptext.begin(), reptext.end(), (ostart + (repstart - start)));
		//std::copy(repend, end, (ostart + (repstart - start) + reptext.size()) );
		region_out.flush();
		*/

	} catch (const std::exception &err)
	{
		std::cerr << err.what() << std::endl;
		return 1;
	}

}

