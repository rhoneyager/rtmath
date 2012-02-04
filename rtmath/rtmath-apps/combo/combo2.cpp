#include <iostream>
#include <cmath>
#include <fstream>
#include <set>
#include <map>
#include <string>
#include <sstream>
#include <vector>
#include <memory>
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
		bool dryrun = true;
		bool outputExists = false;
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
							cin >> clobber;
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
		ofstream out(outfile.c_str());

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
