#include <cmath>
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
//#include "../../rtmath/rtmath/rtmath.h"
#include "../../rtmath/rtmath/error/debug.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace boost::uuids;

	try {
		cerr << "rtmath-uuid\n\n";
		rtmath::debug::appEntry(argc, argv);

		namespace po = boost::program_options;

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("num,n", po::value<size_t>()->default_value(1),"specify number of uuids to generate");

		po::positional_options_description p;
		p.add("num",1);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);

		bool flag = false;

		if (vm.count("help")) {
			cerr << desc << "\n";
			return 1;
		}

		int numUuid = vm["num"].as<size_t>();

		// Loop and generate the uuids, printing 
		// output to the console

		random_generator gen;
		for (int i=0;i<numUuid;i++)
		{
			uuid u = gen();
			cout << u << endl;
		}
	}
	catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}
