// Program to take a shape file and select a slice along the x, y or z axes

#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <rtmath/rtmath.h>
#include <set>
#include <vector>
#include <string>
#include <memory>

void doHelp();

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	try {
		cerr << "rtmath-sector" << endl;
		if (argc == 1) doHelp();
		config::parseParams p(argc, argv);
		string ifstr, ofstr;
		size_t numHeaderLines = 0;
		size_t selColumn = 0;
		double selValue = 0; // Type here is annoying
		// Parameter parsing
		{
			bool flag = false;
			// Help requested
			flag = p.readParam("-h");
			if (flag) doHelp();

			// Input file
			flag = p.readParam("-i", ifstr);
			if (!flag) doHelp();

			// Output file
			flag = p.readParam("-o", ofstr);
			if (!flag) doHelp();

			// Skip header lines
			// Normally -h, but already used for help
			p.readParam<size_t>("-s", numHeaderLines);

			// Column to use
			p.readParam<size_t>("-c", selColumn);

			// Value to select for
			flag = p.readParam<double>("-p", selValue);
			if (!flag) doHelp();
		}
		/*
		cout << "Input: " << ifstr << endl;
		cout << "Output: " << ofstr << endl;
		cout << "Header length: " << numHeaderLines << endl;
		cout << "Column: " << selColumn << endl;
		cout << "Value: " << selValue << endl;
		*/

		// Check for things like file existence
		{
			using namespace boost::filesystem;
			path ip(ifstr);
			path op(ofstr);
			if (!exists(ip))
			{
				cerr << "Input file " << ip << " does not exist!" << endl;
				return 1;
			}
			if (is_directory(ip))
			{
				cerr << "Input " << ip << " is a directory!" << endl;
				return 1;
			}
			if (exists(op))
			{
				// Clobber?

			}
		}

		// Open the files for reading and writing
		// and do processing
		{
			ifstream in(ifstr);
			ofstream out(ofstr);
			// Have in skip header lines
			string lin;
			for (size_t i=0; i<numHeaderLines;i++) 
				std::getline(in,lin);
			// Prepare tokenizer
			typedef boost::tokenizer<boost::char_separator<char> >
				tokenizer;
			boost::char_separator<char> sep(", \t");
			// Process input
			while (in.good())
			{
				getline(in,lin);
				// Tokenize the line
				tokenizer tokens(lin,sep);
				vector<string> lp;
				for (auto it = tokens.begin(); it != tokens.end(); ++it)
					lp.push_back(*it);
				if (lp.size() == 0) continue; // skip empty lines
				if (lp.size() <= selColumn)
				{
					cerr << "Bad file structure\n";
					return 1;
				}
				// Check selColumn value
				using boost::lexical_cast;
				using boost::bad_lexical_cast;
				if (selValue == lexical_cast<double>(lp[selColumn]))
				{
					// Write to output file
					// Leaves an extra tab at the end, but I don't care
					for (auto ot = lp.begin(); ot != lp.end(); ++ot)
						out << *ot << "\t";
					out << endl;
				}
			}
		}
	}
	catch ( rtmath::debug::xError &err )
	{
		using namespace std;
		err.Display();
		cerr << "Error encountered. Terminating." << endl;
		return 1;
	}
}

void doHelp()
{
	using namespace std;
	cerr << "Makes plotting slices of shape.dat easier!\n";
	cerr << "Options:\n";
	cerr << "-i (input file)\n";
	cerr << "-o (output file)\n";
	cerr << "-s (number of lines in file header)\n";
	cerr << "-c (column for slice selection)\n";
	cerr << "-p (value of column to select for)\n";
	cerr << endl;
	exit(1);
}

