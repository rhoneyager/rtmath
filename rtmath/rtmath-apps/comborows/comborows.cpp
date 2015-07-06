/// Program to combine rows of different files based on a common field
#pragma warning( disable : 4996 ) // -D_SCL_SECURE_NO_WARNINGS
#include <iostream>
#include <cmath>
#include <fstream>
#include <set>
#include <map>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <Ryan_Debug/debug.h>
//#include <Ryan_Debug/splitSet.h>
#include "../../rtmath/rtmath/error/debug.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	try {
		cerr << "rtmath-comborows" << endl;

		namespace po = boost::program_options;

		po::options_description desc("Allowed options"), cmdline("Command-line options"),
			config("Config options"), hidden("Hidden options"), oall("all options");
		rtmath::debug::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("output,o", po::value<string>(), 
			 "Specify output file for merge results")
			("inputs,i", po::value<vector<string> >()->multitoken(), 
			 "Specify input files. Files must have the same delimation.")
			("common-columns,c", po::value<vector<size_t> >()->multitoken(),
			 "Columns selected are common to all files, and should "
			 "be incorporated with the output. Indices start at zero.")
			("num-header-lines,s", po::value<size_t>()->default_value(0),
			 "Header lines are ignored in the merge!")
			("missing", po::value<string>()->default_value(""),
			 "If a match cannot be found, this this value will be used to fill.")
			("clobber", "Use this flag if you want to overwrite any "
			 "output file.")

			("delimiter,d", po::value<string>(),
			 "Set the delimiter for input and output. If not specified, defaults "
			 "to \", \\t\". First delimeter is used in output.")
			("delim-space", "Use spaces in the delimeter.")
			("delim-tab", "Use tabs in the delimeter.")

			("preserve-col-names", po::value<bool>()->default_value(true), 
			"Keep column names exactly as-is.")
			
			("verbose,v", "Get extra runtime information and a summary of how "
			 "the command line has been parsed.")
			("test", "Do a dry run only. Don't actually read / write files.")
			;

		po::positional_options_description p;
		p.add("inputs",-1);

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);

		using std::cerr;
		using std::endl;

		vector<string> rawinputs; // May be expanded by os.

		auto doHelp = [=](const std::string& message) {
			if (message.size())
				std::cerr << message << endl << endl;
			std::cerr << desc << "\n";
			exit(2);
		};
		if (vm.count("help") || argc == 1) doHelp("");

		vector<string> infiles;
		string outfile;
		string inscolheader;
		bool doInsert = false;
		size_t numHeaderLines = 0;
		bool clobber = false;
		bool askClobber = true;
		bool verbose = false;
		bool dryrun = false;
		bool outputExists = false;
		bool preserveNames = false;
		vector<size_t> commonCols;
		string missingValue;
		string outSep = "\t";
		// Parse console parameters, including the files to merge
		// Can merge all files of a given extension in a directory
		// and can merge individually-selected files
		// However, there is just one output file
		{
			bool flag = false;

			// Read in common columns
			// Using a more interesting use of parseParams
			if (vm.count("common-columns")) 
				commonCols = vm["common-columns"].as<vector<size_t> >();
			
			// Get number of header lines. If not specified, assume no header
			numHeaderLines = vm["num-header-lines"].as<size_t>();

			// Keep column names exactly as-is?
			preserveNames = vm["preserve-col-names"].as<bool>();

			// Set missing value string
			missingValue = vm["missing"].as<string>();

			// Separate with commas?
			if (vm.count("delimiter"))
			{
				outSep = vm["delimiter"].as<string>();
				if (vm.count("delim-space"))
					outSep.append(" ");
				if (vm.count("delim-tab"))
					outSep.append("\t");
			} else {
				if (vm.count("delim-space"))
					outSep = " ";
				if (vm.count("delim-tab"))
					outSep = "\t";
			}

			// Doing just a dry run?
			if (vm.count("test")) dryrun = true;

			// Verbose output?
			if (vm.count("verbose")) verbose = true;

			// Output file
			if (!vm.count("output")) doHelp("Must specify output file!");
			outfile = vm["output"].as<string>();

			// Input file specification
			// Gets special treatment later to allow for wildcards
			// So, place in a temporary set
			vector<string> sInFiles; // Vector, not set for call
			if (!vm.count("inputs")) doHelp("Must specify input file(s)!");
			sInFiles = vm["inputs"].as<vector<string> >();

			// Clobber output
			if (!vm.count("clobber")) askClobber = true;
			else { askClobber = false; clobber = true; }
			// Check file existence here. If no clobber, die now
			{
				using namespace boost::filesystem;
				path opath(outfile);
				if (exists(opath))
				{
					cerr << "Output file " << outfile << " already exists.\n";
					outputExists = true;
					if ((!clobber) && (!dryrun))
					{
						if (!askClobber)
						{
							if (clobber)
							{
								cerr << "File will be overwritten.\n";
							} else {
								cerr << "Clobbering is turned off. Terminating.\n";
								exit(1);
							}
						} else {
							cerr << "\nOverwrite file? (y/n) ";
							string doClobber;
							cin >> doClobber;
							std::transform(doClobber.begin(), doClobber.end(), 
								doClobber.begin(), ::tolower);
							if (doClobber == "1" || doClobber == "y" || doClobber == "yes")
								clobber = true;
							if (!clobber)
							{
								cerr << "Terminating.\n";
								exit(1);
							}
						}
					}
						
				}
			}


			// Validate input files beforehand
			// Note, I'm relying on shell globbing. It's too annoying to
			// implement myself.
			// Also, will avoid duplicates (hopefully) by looking at absolute paths
			infiles = sInFiles;
			/*
			for (auto it = sInFiles.begin(); it != sInFiles.end(); it++)
			{
				using namespace boost::filesystem;
				path f(*it);
				path fabs = absolute(f);
				if (exists(fabs))
					if (!is_directory(fabs))
						if (infiles.count(fabs.string()) == 0)
							infiles.insert(fabs.string());
			}
			*/
		}



		// If verbose, say what I am going to do
		if (verbose || dryrun)
		{
			cerr << "Command-line arguments parsed." << endl;
			cerr << "Output file: " << outfile << endl;
			if (outputExists)
			{
				cerr << "Output already exists! ";
				if (clobber)
					cerr << "File will be overwritten\n";
				else
					cerr << "Will not overwrite without appropriate selection.\n";
			} else {
				cerr << "No file detected at this location.\n";
			}
			cerr << "Number of header lines: " << numHeaderLines << endl;
			cerr << "Delimeter: " << outSep << "*****" << endl;
			if (preserveNames) cerr << "Preserving exact column names.\n";
			else cerr << "Not preserving exact column names.\n";
			cerr << "Common columns:";
			for (const auto &c : commonCols)
				cerr << " " << c;
			cerr << endl;
			cerr << "Missing value: " << missingValue << endl;

			cerr << "Input files:" << endl;
			for (auto it = infiles.begin(); it != infiles.end(); ++it)
			{
				cerr << "\t" << *it << endl;
			}
			if (dryrun)
				cerr << "Doing a dry run.\n";
		}

		cerr << endl;

		if (dryrun)
		{
			cerr << "Dry run. Terminating.\n";
			exit(0);
		}
		/////////////////////////////////////////////////////////////////////////////////


		// Yay! The parameters have been understood. Open the files for input and output.

		// To hold the loaded data
		// The internal map is the line mapping
		std::vector<std::multimap<std::string, std::string> > lines(infiles.size());
		std::vector<std::string> headers(infiles.size());
		std::vector<std::string> lastheaders(infiles.size());

		//typedef boost::tokenizer<boost::char_separator<char> >
		//	tokenizer;
		//boost::char_separator<char> sep(outSep.c_str());

		// Load in each file, and populate the line mapping
		for (size_t nf=0;nf<infiles.size(); ++nf)
		{
			const auto &f = infiles[nf];
			ifstream in(f.c_str());
			string lin;
			for (size_t i=0; i<numHeaderLines-1; ++i)
			{
				std::getline(in,lin);
				headers.at(nf).append(lin);
			}
			if (numHeaderLines)
			{
				std::getline(in,lin);
				// Remove the trailing line break
				while (size_t found = lin.find_first_of('\n') != string::npos)
				{ lin.erase(found); }
				lastheaders.at(nf) = lin;
			}
			// Main body processing
			while (in.good())
			{
				std::getline(in,lin);
				if (!lin.size()) continue;
				vector<string> strs;
				boost::split(strs, lin, boost::is_any_of(outSep.c_str()));
				string key = strs[commonCols[nf]];
				lines[nf].emplace(std::pair<std::string, std::string>(key, lin));
			}
		}

		// These sets hold the loaded data
		std::map<size_t,std::vector<string> > common;
		std::map<string, unique_ptr<map<size_t,vector<string> > > > merge;

		
		// Open the output file and write the header
		outSep = string(1, outSep.at(0));
		ofstream out(outfile.c_str());
		for (const auto &h : headers) out << h;
		for (size_t i=0; i < lastheaders.size(); ++i)
		{
			out << lastheaders[i];
			if (i != lastheaders.size() - 1) out << outSep;
		}
		out << endl;

		// Using the first file as the dictionary
		// Note: if the dictionary does not contain a key, it is not written to the output, 
		// even if the other files have it.
		for (const auto & line : lines[0])
		{
			out << line.second;
			for (size_t i=1; i<lines.size(); ++i)
			{
				if (lines[i].count(line.first) == 0) continue; // TODO: fill in empty values
				auto it = lines[i].equal_range(line.first);
				for (auto ot = it.first; ot != it.second; ++ot)
				{
					out << outSep << ot->second;
				}
			}
			out << endl;
		}



		cerr << "Merge successful." << endl;

	} catch (std::exception &err)
	{
		std::cerr << err.what() << std::endl;
		return 1;
	}

}

