#pragma once

#include <set>
#include <string>
#include <vector>

class options {
public:
	options()
	{
		merge = 0;
		numHeaderLines = 0;
	}
	std::set<int> preserve;
	int merge;
	std::string dir;
	std::string outfile;
	int numHeaderLines;

	void parseOptions(int argc, char** argv)
	{
		using namespace std;
		for (size_t i=1;i<argc;i++)
		{
			// Iterate through looking for options
			string in(argv[i]);
			if (in == "-d")
			{
				i++;
				dir = string(argv[i]);
			}
			if (in == "-o")
			{
				i++;
				outfile = string(argv[i]);
			}
			if (in == "-h")
			{
				i++;
				int nc = atoi(argv[i]);
				preserve.insert(nc);
			}
			if (in == "-m")
			{
				i++;
				int nc = atoi(argv[i]);
				merge = nc;
			}
			if (in == "-n")
			{
				i++;
				int nc = atoi(argv[i]);
				numHeaderLines = nc;
			}
		}
	}
};

class parsecsv {
public:
	parsecsv() {}
	parsecsv(const std::string &linein)
	{
		parse(linein);
	}
	void parse(const std::string &linein)
	{
		using namespace std;
		istringstream ss( linein );
		while (ss)
		{
		  string s;
		  if (!getline( ss, s, ',' )) break;
		  parsed.push_back( s );
		}
	}
	std::vector<std::string> parsed;
	void print() const
	{
		using namespace std;
		for (size_t i=0;i<parsed.size();i++)
			cout << parsed[i] << ", ";
		cout << endl;
	}
};
