#include <memory>
#include <iostreams>
#include "../../rtmath/rtmath/rtmath.h"

void doHelp();

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace rtmath::debug;
	using namespace rtmath::config;
	try {
		cerr << "Parameter expansion program" << endl;
		rtmath::debug::appEntry(argc,argv);
		rtmath::config::parseParams p(argc,argv);

		if (p.readParam("-h")) doHelp();
		if (!(p.readParam("-s"))) doHelp();

		std::string src, nodename;
		bool mapAliases;
		p.readParam<std::string>("-s",src);
		mapAliases = p.readParam<std::string>("-k",nodename);

		// Will attempt to load a parameter and will perform expansion based on 
		// paramset expansion from key/value combinations found in rtmath.conf

		// config::loadRtconfroot is called during appEntry...
		std::shared_ptr<configsegment> root = rtmath::config::getRtconfRoot();
		std::shared_ptr<configsegment> node;
		if (mapAliases)
			node = root->findSegment(nodename);

		paramSet<double>::aliasmap aliases;
		if (mapAliases)
			node->listKeys(aliases);
		paramSet<double> expander(src, &aliases);

		// expander now has the full expanded list of everything
		cout << "Expansion of " << src << ":\n\n";

		for (auto it = expander.begin(); it != expander.end(); it++)
		{
			cout << *it << "\t";
		}
		cout << endl << endl;
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		return 1;
	}
	catch (std::exception &e)
	{
		cerr << "exception caught: " << e.what() << endl;
		return 2;
	}
	return 0;
}

void doHelp()
{
	using namespace std;
	cerr << "-s (string)\n";
	cerr << "\tThe string to be expanded.\n";
	cerr << "-k (key location)\n";
	cerr << "\tThe location of the alias table for expansion (OPTIONAL)" << endl;
	cerr << endl;
	exit(0);
}

