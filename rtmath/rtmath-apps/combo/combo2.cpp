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
#include <boost/program_options.hpp>
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/splitSet.h>
#include "../../rtmath/rtmath/error/debug.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	try {
		cerr << "rtmath-combo" << endl;

		namespace po = boost::program_options;

		po::options_description desc("Allowed options"), cmdline("Command-line options"),
			config("Config options"), hidden("Hidden options"), oall("all options");
		rtmath::debug::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("output,o", po::value<string>(), 
			 "Specify output file for merge results")
			("inputs,i", po::value<vector<string> >(), 
			 "Specify input files. Files must have the same delimation.")
			("common-columns,c", po::value<vector<string> >(),
			 "Columns selected are common to all files, and should "
			 "be incorporated with the output. Indices start at zero.")
			("merge-columns,m", po::value<vector<string> >(),
			 "These columns get merged into the output. Again, indices "
			 "begin at zero.")
			("insert-column", 
			 "Insert a column filled with the 'missing value' specifier into "
			 "the output. Useful when tagging input data")
			("insert-column-header", po::value<string>()->default_value("newcol"),
			 "Specify the new column name for column insertion.")
			("num-header-lines,s", po::value<size_t>()->default_value(0),
			 "Header lines are ignored in the merge!")
			("missing", po::value<string>()->default_value("---"),
			 "If the files have different numbers of rows, then the output "
			 "will have missing values. Use this flag to set the resultant "
			 "column placeholder text.")
			("clobber", "Use this flag if you want to overwrite any "
			 "output file.")

			("delimiter,d", po::value<string>(),
			 "Set the delimiter for input and output. If not specified, defaults "
			 "to \", \\t\". First delimeter is used in output.")
			("delim-space", "Use spaces in the delimeter.")
			("delim-tab", "Use tabs in the delimeter.")

			("preserve-col-names", "Keep column names exactly as-is.")
			
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

		vector<string> rawinputs; // May be expanded by os.

		auto doHelp = [=](const std::string& message) {
			if (message.size())
				std::cerr << message << endl << endl;
			std::cerr << desc << "\n";
			exit(2);
		};
		if (vm.count("help") || argc == 1) doHelp("");

		set<string> infiles;
		string outfile;
		string inscolheader;
		bool doInsert = false;
		size_t numHeaderLines = 0;
		set<size_t> common_cols;
		set<size_t> merge_cols;
		bool clobber = false;
		bool askClobber = true;
		bool verbose = false;
		bool dryrun = false;
		bool outputExists = false;
		bool preserveNames = false;
		string missingValue;
		string outSep = ",\t ";
		// Parse console parameters, including the files to merge
		// Can merge all files of a given extension in a directory
		// and can merge individually-selected files
		// However, there is just one output file
		{
			bool flag = false;

			// Read in common columns
			// Using a more interesting use of parseParams
			vector<string> sCommonCols;
			if (vm.count("common-columns")) 
				sCommonCols = vm["common-columns"].as<vector<string> >();
			
			// Read in merge columns
			vector<string> sMergeCols;
			if (vm.count("merge-columns")) 
				sMergeCols = vm["merge-columns"].as<vector<string> >();

			if (vm.count("insert-column"))
			{
				doInsert = true;
				if (vm.count("insert-column-header"))
					inscolheader = vm["insert-column-header"].as<string>();
			}

			// Get number of header lines. If not specified, assume no header
			numHeaderLines = vm["num-header-lines"].as<size_t>();

			// Keep column names exactly as-is?
			if (vm.count("preserve-col-names"))
				preserveNames = true;

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


			// Expand and validate arguments
			
			// Handle the common and merged columns in the same manner
			for (auto it = sCommonCols.begin(); it != sCommonCols.end(); it++)
				Ryan_Debug::splitSet::splitSet<size_t>(*it,common_cols);
			for (auto it = sMergeCols.begin(); it != sMergeCols.end(); it++)
				Ryan_Debug::splitSet::splitSet<size_t>(*it,merge_cols);

			// Validate input files beforehand
			// Note, I'm relying on shell globbing. It's too annoying to
			// implement myself.
			// Also, will avoid duplicates (hopefully) by looking at absolute paths
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
			for (auto it = common_cols.begin(); it != common_cols.end(); ++it)
				cerr << " " << *it;
			cerr << endl;
			cerr << "Merge columns:";
			for (auto it = merge_cols.begin(); it != merge_cols.end(); ++it)
				cerr << " " << *it;
			cerr << endl;
			cerr << "Missing value: " << missingValue << endl;
			if (doInsert)
			{
				cerr << "Inserting column:" << endl;
				cerr << "\twith values " << missingValue << endl;
				cerr << "\tand header " << inscolheader << endl;
			}

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

		// These sets hold the loaded data
		std::map<size_t,std::vector<string> > common;
		std::map<string, unique_ptr<map<size_t,vector<string> > > > merge;

		typedef boost::tokenizer<boost::char_separator<char> >
			tokenizer;
		boost::char_separator<char> sep(outSep.c_str()); //",\t ");

		// Populate the common map with an empty vector
		vector<string> cloneVector; 
		for (auto bt = common_cols.begin(); bt != common_cols.end(); ++bt)
			common[*bt]=cloneVector; // Ugly, but effective

		// Loop through the input files
		for (auto it = infiles.begin(); it != infiles.end(); it++)
		{
			ifstream in(*it);
			string lin;
			// Create the file data pools (this file's portion of the merge map)
			unique_ptr< map<size_t,vector<string> > > pFile (new map<size_t,vector<string> >);
			for (auto bt = merge_cols.begin(); bt != merge_cols.end(); ++bt)
				(*pFile)[*bt]=cloneVector; // Ugly, but effective
			
			// Advance through the header
			for (size_t i=0;i<numHeaderLines;i++)
				std::getline(in,lin);

			while (in.good())
			{
				// Begin reading the data line-by-line.
				getline(in,lin);
				tokenizer parser(lin,sep);
				size_t col = 0; // Doesn't like to be near auto on MSVC parser
				for (auto ot=parser.begin(); ot != parser.end(); ++ot, col++)
				{
					// Process common columns, but only for the first file
					if (common_cols.count(col) && it == infiles.begin())
						common[col].push_back(*ot);

					// Do the other columns
					if (merge_cols.count(col))
						(*pFile)[col].push_back(*ot);

				}
			}
			// Add pFile to the merge map
			merge[*it] = std::move(pFile); // pFile is transferred to the container
			// Close the input file
			in.close();
		}



		// Now to output the resultant file, finally.
		ofstream out(outfile.c_str());
		// Unfortunately, I have the column data, but I really need it in rows...
		// Here's how the file is written:
		// Common columns come first, then file-unique columns
		// These columns have a header indicating the file (if not common) and the source column
		// For safety, verify the longest number of rows needed for write, and check _all_
		// bounds.
		size_t maxRows = 0;
		for (auto it = common.begin(); it != common.end(); ++it)
			if (it->second.size() > maxRows) maxRows = it->second.size();
		// Note: these two auto types are different.
		for (auto it = merge.begin(); it != merge.end(); ++it)
			for (auto ot = it->second->begin(); ot != it->second->end(); ++ot)
				if (ot->second.size() > maxRows) maxRows = ot->second.size();

		outSep = string(1, outSep.at(0));

		// Good. The number of lines to write has been determined
		// For file write, note that I'll leave an excess tab at the end. It's easier to code.
		// Write the header
		bool pcom = false;
		for (auto it = common.begin(); it != common.end(); ++it)
		{
			if (pcom) out << outSep;
			if (!preserveNames)
				out << "common-";
			out << it->first;
			pcom = true;
		}
		for (auto it = merge.begin(); it != merge.end(); ++it)
			for (auto ot = it->second->begin(); ot != it->second->end(); ++ot)
			{
				if (pcom) out << outSep;
				if (!preserveNames)
					out << it->first << "-";
				out << ot->first;
				pcom = true;
			}
		if (doInsert)
		{
			if (pcom) out << outSep;
			out << inscolheader;
		}
		out << endl;

		// Write the rows
		for (size_t i=0; i<maxRows;i++)
		{
			pcom = false;
			// First come the common columns
			for (auto it = common.begin(); it != common.end(); ++it)
				if (it->second.size() > i)
				{
					if (pcom) out << outSep;
					out << it->second[i];
					pcom = true;
				} else {
					if (pcom) out << outSep;
					out << missingValue;
					pcom = true;
				}
			// Then come the merged columns
			for (auto it = merge.begin(); it != merge.end(); ++it)
				for (auto ot = it->second->begin(); ot != it->second->end(); ++ot)
					if (ot->second.size() > i)
					{
						if (pcom) out << outSep;
						out << ot->second[i];
						pcom = true;
					} else {
						if (pcom) out << outSep;
						out << missingValue;
						pcom = true;
					}
			// Last will come the inserted column
			if (doInsert)
			{
				if (pcom) out << outSep;
				out << missingValue;
			}
			out << endl;
		}

		// Close the output file
		out.close();

		cerr << "Merge successful." << endl;

	} catch (std::exception &err)
	{
		std::cerr << err.what() << std::endl;
		return 1;
	}

}

