/* This program is designed to process scanning radar data from ARM */

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <algorithm>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include "station.h"
#include "parser.h"
#include "export.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace boost::filesystem;

	try {
		cerr << "ncdc-snowmonitoring-convert\n\n";

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);

		po::options_description desc("Allowed options");

		desc.add_options()
			("help,h", "produce help message")
			("input,i", po::value< vector<string> >()->multitoken(), "input text files")
			("recursive,r", "Select recursive directory input")
			("output,o", po::value< string >(), "output file")
			;

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);  

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << "\n";
			if (message.size())
				cerr << message << "\n";
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");
		if (!vm.count("input")) doHelp("Need to specify input file(s).\n");
		vector<string > inputs = vm["input"].as< vector<string > >();
		string output;
		if (vm.count("output")) output= vm["output"].as< string >();
		else doHelp("Need to specify an output file.\n");

		bool recurse = false;
		if (vm.count("recursive")) recurse = true;
		auto expandFolders = [&](const vector<string> &src, vector<path> &dest)
		{
			dest.clear();
			for (auto s : src)
			{
				using namespace boost::filesystem;
				path p(s);
				if (is_directory(p))
				{
					if (!recurse)
						copy(directory_iterator(p), 
						directory_iterator(), back_inserter(dest));
					else
						copy(recursive_directory_iterator(p,symlink_option::recurse), 
						recursive_directory_iterator(), back_inserter(dest));
				}
				else dest.push_back(p);
			}
		};

		vector<boost::filesystem::path> vinputs;
		expandFolders(inputs, vinputs);

		std::map<int,station> stations;

		for (const auto &si : vinputs)
		{
			// Validate input file
			path pi(si);
			if (!exists(pi)) throw std::string("Missing file ").append(pi.string());
			if (is_symlink(pi)) pi = boost::filesystem::read_symlink(pi);
			if (!exists(pi)) throw std::string("Missing symbolic link location ").append(pi.string());
			if (is_directory(pi)) continue;
			if (pi.filename().string().at(0) == '.') continue;
			if (pi.extension().string() != ".txt") continue;
			if (pi.string().find("snfl.txt") == string::npos) continue;

			cerr << "Input: " << si << endl;
			//cerr << "Input: " << pi.filename() << endl;

			// From the filename, determine the month and year of the observations.
			int month = boost::lexical_cast<int>(pi.filename().string().substr(0,2));
			int year = boost::lexical_cast<int>(pi.filename().string().substr(3,4));

			parse_file(month, year, pi.string(), stations);
		}

		if (!stations.size()) throw std::string("No station data was successfully read!");

		// Calculate start and end dates
		boost::gregorian::date start = stations.begin()->second.startTime;
		boost::gregorian::date end = stations.begin()->second.endTime;
		for (const auto & s : stations)
		{
			//cout << s.second;
			if (start > s.second.startTime) start = s.second.startTime;
			if (end < s.second.endTime) end = s.second.endTime;
		}
		cerr << "Read data for " << stations.size() << " stations.\n";
		cerr << "Start date: " << start << "\nEnd Date: " << end << endl;

		exportToHDF(output, start, end, stations);
		
	}
	catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

