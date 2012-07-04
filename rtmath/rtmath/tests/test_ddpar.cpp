#include <string>
#include <iostream>
#define BOOST_TEST_DYN_LINK
#include "globals.h"
#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <memory>

#include "../rtmath/rtmath.h"

using namespace rtmath;
using namespace std;
using namespace rtmath::config;
using namespace boost::filesystem;

struct ddparloader
{
	ddparloader()
		: defaultKey(false),
		baseExists(false),
		testPar(nullptr)
	{
		// Check that there is a default key
		string sBasePar;
		defaultKey = globals::instance()->cRoot->getVal<string>("ddscat/DefaultFile", sBasePar);
		if (!defaultKey) BOOST_FAIL("ddscat/DefaultFile key missing from testing rtmath.conf");

		// Check that default key points to a file
		string scwd;
		globals::instance()->cRoot->getCWD(scwd);
		pCWD = path(scwd).remove_filename();

		pBasePar = boost::filesystem::absolute(path(sBasePar), pCWD);
		if (!exists(pBasePar)) BOOST_FAIL("ddscat/DefaultFile key refers to nonexistant file");
		if (is_directory(pBasePar)) BOOST_FAIL("ddscat/DefaultFile key points to a directory, not a file");

		// Attempt to load the default
		try {
			testPar = rtmath::ddscat::ddPar::defaultInstance();
		} catch (std::exception &e)
		{
			cerr << e.what() << endl;
			BOOST_FAIL("Unable to load default testing ddscat.par file. Cannot do ddPar tests.");
		}
	}
	~ddparloader()
	{
	}
	bool baseExists;
	bool defaultKey;
	path pBasePar;
	path pCWD;
	rtmath::ddscat::ddPar *testPar;
};


//BOOST_AUTO_TEST_SUITE(test_ddpar);
BOOST_FIXTURE_TEST_SUITE(test_ddpar, ddparloader);
// globals::instance()->

BOOST_AUTO_TEST_CASE(ddpar_defaultLoad)
{
	try {
		rtmath::ddscat::ddPar par(pBasePar.string());
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		BOOST_FAIL("Exception thrown.");
	}
}



BOOST_AUTO_TEST_SUITE_END();

