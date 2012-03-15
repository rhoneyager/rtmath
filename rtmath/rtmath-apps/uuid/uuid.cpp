#include <cmath>
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <boost/tokenizer.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include "../../rtmath/rtmath/rtmath.h"

void doHelp();

int main(int argc, char** argv)
{
	using namespace std;
	using namespace boost::uuids;

	try {
		if (argc == 1) doHelp();
		rtmath::debug::appEntry(argc, argv);
		rtmath::config::parseParams p(argc,argv);

		bool flag = false;

		p.readParam("-h",flag);
		if (flag) doHelp();

		int numUuid;
		flag = p.readParam<int>("-n", numUuid);
		if (!flag) doHelp();
		if (numUuid < 0) doHelp();

		// Loop and generate the uuids, printing 
		// output to the console

		random_generator gen;
		for (int i=0;i<numUuid;i++)
		{
			uuid u = gen();
			cout << u << endl;
		}
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		cerr << endl;
#ifdef _WIN32
		std::getchar();
#endif
		return 1;
	}
	return 0;
}


void doHelp()
{
	using namespace std;
	cout << "rtmath-uuid\n\n";
	cout << "A progran for generating uuids for use with\n";
	cout << "database programs in the rtmath library\n";
	cout << "Options:\n";
	cout << "-n (number of uuids to generate)\n";
	cout << "-h\n";
	cout << "\tProduce this help message.\n";
	cout << endl << endl;
#ifdef _WIN32
	std::getchar();
#endif
	exit(1);
}

