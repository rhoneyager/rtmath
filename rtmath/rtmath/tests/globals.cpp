#include <boost/filesystem.hpp>
#define BOOST_TEST_NO_MAIN
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>
#include <exception>
#include <string>
#include <iostream>
#include <sstream>

#include "globals.h"
#include "../rtmath/rtmath.h"

globals::globals()
{
	BOOST_TEST_MESSAGE( "Loading globals" );
	instance() = this;
	try {
		findDirs();
		loadConfig();
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		BOOST_FAIL("Test threw an error.");
	}
	catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		BOOST_FAIL("Test threw an error.");
	}
}

globals::~globals()
{
	BOOST_TEST_MESSAGE( "Unloading globals" );
}

void globals::findDirs()
{
	using namespace std;
	using namespace boost::filesystem;

	// Find test data
	{
		const size_t sCand = 2;
		const char* cand[sCand] = {
			"./data", // MSVC reg soln build
			"../rtmath/rtmath/tests/data" // cmake
		};
		bool done = false;
		for (size_t i=0; i<sCand && !done; i++)
		{
			pTestData = path(cand[i]);
			if (exists(pTestData)) done = true;
		}
		if (!done)
			BOOST_FAIL("Cannot find test data directory.");
	}

	// Find full data
	{
		const size_t sCand = 2;
		const char* cand[] = {
			"../data", // MSVC reg soln
			"../rtmath/rtmath/data"
		};
		bool done = false;
		for (size_t i=0; i<sCand && !done; i++)
		{
			pFullData = path(cand[i]);
			if (exists(pFullData)) done = true;
		}
		if (!done)
			BOOST_FAIL("Cannot find full data directory.");
	}

	pProfiles = pFullData / "profiles/";
	pRtconf = pTestData / "rtmath.conf";
}

void globals::loadConfig()
{
	using namespace std;
	using namespace boost::filesystem;
	if (!exists(pRtconf))
		throw rtmath::debug::xMissingFile(pRtconf.string().c_str());
	cRoot =  rtmath::config::loadRtconfRoot(pRtconf.string());
}

globals*& globals::instance()
{
	static globals* s_inst = nullptr;
	return s_inst;
}
