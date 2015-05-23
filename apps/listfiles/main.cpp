#include <memory>
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
namespace fs = ::boost::filesystem;

static fs::path relativeTo(fs::path from, fs::path to)
{
	// Start at the root path and while they are the same then do nothing then when they first
	// diverge take the remainder of the two path and replace the entire from path with ".."
	// segments.
	fs::path::const_iterator fromIter = from.begin();
	fs::path::const_iterator toIter = to.begin();

	// Loop through both
	while (fromIter != from.end() && toIter != to.end() && (*toIter) == (*fromIter))
	{
		++toIter;
		++fromIter;
	}

	fs::path finalPath;
	while (fromIter != from.end())
	{
		finalPath /= "..";
		++fromIter;
	}

	while (toIter != to.end())
	{
		finalPath /= *toIter;
		++toIter;
	}

	return finalPath;
}

int main(int argc, char** argv)
{
	using namespace std;
	try {
		cerr << "Program to find all files matching a pattern under a certain directory" << endl;

		namespace po = boost::program_options;
		po::options_description desc("Allowed options"), cmdline("Command-line options"),
		config("Config options"), hidden("Hidden options"), oall("all options");
		
		cmdline.add_options()
			("help,h", "Produce help message")
			("recurse,r", "Recurse through directories")
			("follow-symlinks", "Follow symlinks")
			("base,b", po::value<std::vector<std::string> >(), "Base directories")
			("extension,e", po::value<std::vector<std::string> >()->multitoken(), "Extensions to match")
			("preamble", po::value<string>(), "Preamble for use in scripting")
			("relative", po::value<string>(), "Specify directory that all output paths should be relative to")
			;

		po::positional_options_description p;
		
		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		auto doHelp = [&]()
		{
			cerr << desc << std::endl;
			exit(1);
		};

		if (vm.count("help")) doHelp();

		if (!vm.count("base")) doHelp();
		if (!vm.count("extension")) doHelp();

		vector<string> bases = vm["base"].as<vector<string> >();
		vector<string> exts = vm["extension"].as<vector<string> >();

		string preamble;
		if (vm.count("preamble")) preamble = vm["preamble"].as<string>();
		string relative;
		if (vm.count("relative")) relative = vm["relative"].as<string>();

		bool recurse = false;
		if (vm.count("recurse")) recurse = true;
		bool follow = false;
		if (vm.count("follow-symlinks")) follow = true;

		vector<string> expanded;

		using namespace boost::filesystem;

		path pRelative(relative);

		for (const auto &b : bases) {
			path p(b);
			std::vector<path> dest;
			if (is_directory(p))
			{
				if (!recurse)
					copy(directory_iterator(p),
					directory_iterator(), back_inserter(dest));
				else
					copy(recursive_directory_iterator(p, symlink_option::recurse),
					recursive_directory_iterator(), back_inserter(dest));
			}
			else dest.push_back(p);
			for (const auto & i : dest) {
				for (const auto & j : exts) {
					if (i.extension().string() == j) {
						path irel = i;
						if (relative.size())
							irel = relativeTo(pRelative, i);
						expanded.push_back(irel.string());
					}
				}
			}
		}

		if (preamble.size()) cout << preamble;
		for (auto it = expanded.begin(); it != expanded.end(); it++)
		{
			if (it != expanded.begin() || preamble.size()) cout << " ";
			cout << *it;
		}
		cout << endl;
	}
	catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}


