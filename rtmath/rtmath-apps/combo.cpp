#include <iostream>
#include <cmath>
#include <fstream>
#include <set>
#include <map>
#include <string>
#include <sstream>
#include <vector>
#include <boost/filesystem.hpp>
#include "options.h"

/* program to parse several csv files, extract one column, and then reoutput the results */

//void printHelp();

int main(int argc, char** argv)
{
	using namespace std;
	using namespace boost::filesystem;

	//if (argc < 2) printHelp();

	if (argc < 2) return 1;
	// Parse options
	options ops;
	ops.parseOptions(argc,argv);

	boost::filesystem::path p(ops.dir.c_str()), dir, ddfile;
	if (!exists(p)) throw;
	if (is_regular_file(p))
	{
		// Need to extract the directory path
		dir = p.parent_path();
	}
	if (is_directory(p))
	{
		// Need to extract the path for ddscat.par
		dir = p;
	}
	cout << "Directory: " << dir << endl;

	// Use boost to select and open all files in path
	// Iterate through each .fml file and load
	vector<path> files;
	copy(directory_iterator(dir), directory_iterator(), back_inserter(files));

	cout << "There are " << files.size() << " files in the directory." << endl;
	// Iterate through file list for .csv files
	vector<path>::const_iterator it;
	size_t counter = 0;
	for (it = files.begin(); it != files.end(); it++)
	{
		//cout << *it << "\t" << it->extension() << endl;
		if (it->extension().string() == string(".csv") )
		{
			//ddOutputSingle news(it->string());
			counter++;
		}
	}
	cout << "Of these, there are " << counter << " csv files." << endl;

	// Create the arrays for file output
	std::vector< std::vector<double> > common;
	std::vector< std::vector<double> > columns;
	std::vector< std::string > header;
	columns.resize(counter);
	common.resize(ops.preserve.size());
	header.resize(ops.preserve.size());

	cout << "Reading files...\n";

	bool headerDone = false;
	bool headerLoading = false;
	int filenum = 0;
	for (it = files.begin(); it != files.end(); it++)
		{
			if (it->extension().string() == string(".csv") )
			{
				ifstream in(it->string().c_str());
				string lin;
				// Read in the header lines and ignore
				for (size_t i=0;i<ops.numHeaderLines;i++)
					std::getline(in,lin);
				// Now, loop to completion and parse those lines!
				while (!in.eof())
				{
					std::getline(in,lin);
					parsecsv ps(lin);
					if (ps.parsed.size() < 2) continue;
					//ps.print();
					// Add standard columns
					double num = atof(ps.parsed[ops.merge].c_str() );
					//cout << num << endl;
					columns[filenum].push_back( num );
					//cout << columns[filenum].size() << endl;
					// If header not yet done, add in header columns
					if (!headerDone)
					{
						headerLoading = true;
						std::set<int>::const_iterator ot;
						int i;
						for(ot = ops.preserve.begin(), i=0; ot !=ops.preserve.end(); ot++, i++)
						{
							double a = atof(ps.parsed[*ot].c_str());
							//cout << a << endl;
							common[i].push_back( a );
							//cout << common[*ot].size() << endl;
						}
					}
				}
				if (headerLoading) headerDone = true;
				// Add filename to the header field
				header.push_back(it->string().c_str());
				filenum++;
				//cout << columns[filenum-1].size() << endl;
			}
	}

	// Okay, now write out the data
	cout << "Writing data\n";
	ofstream out(ops.outfile.c_str());
	// Header
	//  Common columns
	for(size_t i=0;i<ops.preserve.size();i++)
		out << ",";
	//  Filenames
	for(size_t i=0;i<counter;i++)
		out << header[i] << ",";
	out << endl;

	// Data
	int numRows = columns[0].size();
	for (size_t i=0;i<numRows;i++)
	{
		// Output common stuff
		//for (size_t j=0;j<common[i].size();j++)
			//out << ",";
			//out << common[j][i] << ",";

		// Output merged stuff
		for (size_t j=0;j<counter;j++)
		{
			out << columns[j][i] << ",";
		}
		out << endl;
	}

	return 0;
}
