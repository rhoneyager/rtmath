#include <iostream>
#include <cmath>
#include <fstream>
#include <set>
#include <map>
#include <string>
#include <sstream>
#include <vector>
#include <memory>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include "../../rtmath/rtmath/rtmath.h"

// combo replacement program, since combo has annoying undebugged memory bounds errors

void doHelp();

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	try {
		cerr << "rtmath-combo" << endl;
		if (argc == 1) doHelp();
		config::parseParams p(argc,argv);
		set<string> infiles;
		string outfile;
		size_t numHeaderLines = 0;
		set<size_t> common_cols;
		set<size_t> merge_cols;
		bool clobber = false;
		bool askClobber = true;
		bool verbose = false;
		bool dryrun = false;
		bool outputExists = false;
		string missingValue = "---";
		string outSep = "\t";
		// Parse console parameters, including the files to merge
		// Can merge all files of a given extension in a directory
		// and can merge individually-selected files
		// However, there is just one output file
		{
			bool flag = false;

			// Read in common columns
			// Using a more interesting use of parseParams
			vector<string> sCommonCols;
			flag = p.readParam<string>("-c",sCommonCols);
			//if (!flag) doHelp();
			// Parse all strings and write values to set common_cols

			// Read in merge columns
			vector<string> sMergeCols;
			flag = p.readParam<string>("-m",sMergeCols);
			if (!flag) doHelp(); // Otherwise, why is this program even running?

			// Get number of header lines. If not specified, assume no header
			p.readParam<size_t>("-s",numHeaderLines);

			// Set missing value string
			p.readParam<string>("--missing",missingValue);

			// Separate with commas?
			flag = p.readParam("--sepcommas");
			if (flag) outSep = ",";

			// Doing just a dry run?
			if (!dryrun) // Will set to false at later development stage
				dryrun = p.readParam("--test");

			// Output file
			flag = p.readParam<string>("-o",outfile);
			if (!flag) doHelp();

			// Input file specification
			// Gets special treatment later to allow for wildcards
			// So, place in a temporary set
			vector<string> sInFiles; // Vector, not set for call
			flag = p.readParam<string>("-i",sInFiles);
			if (!flag) doHelp();

			// Clobber output
			askClobber = !p.readParam("--clobber", clobber);
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
				rtmath::config::splitSet<size_t>(*it,common_cols);
			for (auto it = sMergeCols.begin(); it != sMergeCols.end(); it++)
				rtmath::config::splitSet<size_t>(*it,merge_cols);

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
			cerr << "Common columns:";
			for (auto it = common_cols.begin(); it != common_cols.end(); ++it)
				cerr << " " << *it;
			cerr << endl;
			cerr << "Merge columns:";
			for (auto it = merge_cols.begin(); it != merge_cols.end(); ++it)
				cerr << " " << *it;
			cerr << endl;
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
		boost::char_separator<char> sep(",\t ");

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
		// Good. The number of lines to write has been determined
		// For file write, note that I'll leave an excess tab at the end. It's easier to code.
		// Write the header
		for (auto it = common.begin(); it != common.end(); ++it)
			out << "common-" << it->first << outSep;
		for (auto it = merge.begin(); it != merge.end(); ++it)
			for (auto ot = it->second->begin(); ot != it->second->end(); ++ot)
				out << it->first << "-" << ot->first << outSep;
		out << endl;
		// Write the rows
		for (size_t i=0; i<maxRows;i++)
		{
			// First come the common columns
			for (auto it = common.begin(); it != common.end(); ++it)
				if (it->second.size() > i)
				{
					out << it->second[i] << outSep;
				} else {
					out << missingValue << outSep;
				}
			// Then come the merged columns
			for (auto it = merge.begin(); it != merge.end(); ++it)
				for (auto ot = it->second->begin(); ot != it->second->end(); ++ot)
					if (ot->second.size() > i)
					{
						out << ot->second[i] << outSep;
					} else {
						out << missingValue << outSep;
					}
			out << endl;
		}

		// Close the output file
		out.close();

		cerr << "Merge successful." << endl;

	} catch (rtmath::debug::xError &err)
	{
		err.Display();
		return 1;
	}

}

void doHelp()
{
	using namespace std;
	cerr << "This program is designed to merge several datafiles with the "
		"same overall structure.\n";
	cerr << "It is the user's responsibility to ensure that the axes are aligned.\n";
	cerr << "Options:\n";
	cerr << "-o output file (required)\n";
	cerr << "\tSpecifies the output file\n";
	cerr << "-i input files (required)\n";
	cerr << "\tMultiple input files may be selected, either through\n";
	cerr << "\tinvoking -i multiple times, or by using a regular expression.\n";
	cerr << "-c (common columns)\n";
	cerr << "\tColumns selected are common to all files, and should \n";
	cerr << "\tbe incorporated into the output. Indices start at zero.\n";
	cerr << "-m (merge columns)\n";
	cerr << "\tThese columns get merged into the output.\n";
	cerr << "\tAgain, indices begin at zero.\n";
	cerr << "-s (number of header lines)\n";
	cerr << "\tHeader lines are ignored in the merge!\n";
	cerr << "--missing (missing specifier)\n";
	cerr << "\tIf the files are have different numbers of rows, then the output\n";
	cerr << "\twill have missing values. Use this flag to set the missing value string.\n";
	cerr << "--sepcommas\n";
	cerr << "\tUse commas instead of tabs in file output.\n";
	cerr << "--clobber\n";
	cerr << "\tUse this flag if an output file already exists and you\n";
	cerr << "\twant to suppress the overwrite question.\n";
	cerr << "--verbose\n";
	cerr << "\tGet extra runtime information and a summary of how\n";
	cerr << "\tthe command line has been parsed.\n";
	cerr << "--test\n";
	cerr << "\tDo a dry run only. Don't actually read or write any files.\n";
	cerr << "-h\n";
	cerr << "\tPrint this help message.\n";
	cerr << endl;
	exit(1);
}
