/// Program to read a table and perform lowess regression on it.
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
#include "lowess.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	try {
		cerr << "rtmath-lowess" << endl;

		namespace po = boost::program_options;

		po::options_description desc("Allowed options"), cmdline("Command-line options"),
			config("Config options"), hidden("Hidden options"), oall("all options");
		rtmath::debug::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("output,o", po::value<string>(),
			 "Specify output file")
			("input,i", po::value<string>(), "Specify input file")
			("dependent-axis,y", po::value<int>()->default_value(2), "Specify dependent axis "
			 "column, with counting starting at 1.")
			("independent-axis,x", po::value<int>()->default_value(1), "Specify independent axis "
			 "column, with counting starting at 1.")
			("num-header-lines,s", po::value<size_t>()->default_value(0),
			 "Header lines are ignored in the merge!")
			("delimiter,d", po::value<string>(),
			 "Set the delimiter for input and output. If not specified, defaults "
			 "to \", \\t\". First delimeter is used in output.")
			("delim-space", "Use spaces in the delimeter.")
			("delim-tab", "Use tabs in the delimeter.")
			("smoothing,f", po::value<double>()->default_value(0.5),
			 "Smoothing parameter for lowess")
			("nsteps,n", po::value<long>()->default_value(2),
			 "Nsteps for lowess")
			("delta", po::value<double>()->default_value(0),
			 "Delta for lowess")
			;

		po::positional_options_description p;

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

		string sInput;
		string outfile;
		string inscolheader;
		size_t numHeaderLines = 0;
		string missingValue;
		string outSep = ",\t ";
		bool flag = false;
		int icol = 0;
		int ocol = 0;
		double smoothing = vm["smoothing"].as<double>();
		long nsteps = vm["nsteps"].as<long>();
		double delta = vm["delta"].as<double>();
		if (vm.count("dependent-axis")) ocol = vm["dependent-axis"].as<int>();
		else doHelp("Must specify dependent axis");
		if (vm.count("independent-axis")) icol = vm["independent-axis"].as<int>();
		else doHelp("Must specify independent axis");
		// Get number of header lines. If not specified, assume no header
		numHeaderLines = vm["num-header-lines"].as<size_t>();

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

		if (!vm.count("output")) doHelp("Must specify output file!");
		outfile = vm["output"].as<string>();
		if (!vm.count("input")) doHelp("Must specify input file!");
		sInput = vm["input"].as<string >();

		std::vector<double> idata, odata;
		idata.reserve(100000);
		odata.reserve(100000);
		//Eigen::Array<double, Eigen::Dynamic, 2> data;

		typedef boost::tokenizer<boost::char_separator<char> >
			tokenizer;
		boost::char_separator<char> sep(outSep.c_str()); //",\t ");

		ifstream in(sInput);
		string lin;
		// Advance through the header
		for (size_t i=0;i<numHeaderLines;i++)
			std::getline(in,lin);

		while (in.good())
		{
			// Begin reading the data line-by-line.
			getline(in,lin);
			tokenizer parser(lin,sep);
			int col = 1; // Doesn't like to be near auto on MSVC parser
			for (auto ot=parser.begin(); ot != parser.end(); ++ot, col++)
			{
				if (ocol == col) odata.push_back(boost::lexical_cast<double>(*ot));
				if (icol == col) idata.push_back(boost::lexical_cast<double>(*ot));
			}
		}
		in.close();

		// Now to output the resultant file, finally.
		ofstream out(outfile.c_str());
		outSep = string(1, outSep.at(0));

		// Good. The number of lines to write has been determined
		// For file write, note that I'll leave an excess tab at the end. It's easier to code.
		// Write the header
		out << "x" << outSep << "y_raw" << outSep << "y_regressed" << outSep
			<< "Residual_Weight" << outSep << "Residual" << std::endl;

		// Need to first sort the data
		std::vector<std::pair<double, double> > data;
		data.reserve(idata.size());
		for (size_t i=0; i<idata.size();i++)
			data.push_back(std::pair<double, double>(idata[i], odata[i]));
		std::sort(data.begin(), data.end(), [](
			std::pair<double, double> lhs, std::pair<double, double> rhs) {
				if (lhs.first != rhs.first)
					return lhs.first < rhs.first;
				return lhs.second < rhs.second;
			});
		for (size_t i=0; i<idata.size();i++) {
			idata[i] = data[i].first;
			odata[i] = data[i].second;
		}

		// Now run lowess
//void lowess(const vector<double> &x, const vector<double> &y, double f, long nsteps,
//	double delta, vector<double> &ys, vector<double> &rw, vector<double> &res);
		std::vector<double> ys, rw, residuals;
		ys.resize(idata.size());
		rw.resize(idata.size());
		residuals.resize(idata.size());
		lowess(idata, odata, smoothing, nsteps, delta, ys, rw, residuals);
		// Write the rows
		for (size_t i=0; i<idata.size();i++)
		{
			out << idata[i] << outSep << odata[i] << outSep
				<< ys[i] << outSep << rw[i] << outSep
				<< residuals[i] << std::endl;
		}

	} catch (std::exception &err)
	{
		std::cerr << err.what() << std::endl;
		return 1;
	}

}

