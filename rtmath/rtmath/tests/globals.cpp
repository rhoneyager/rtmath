#include <boost/filesystem.hpp>
#define BOOST_TEST_NO_MAIN
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>
#include <exception>
#include <string>
#include <iostream>
#include <sstream>
#include <set>

#include "globals.h"
#include "../rtmath/error/error.h"
#include "../rtmath/config.h"
#include "../rtmath/ddscat/ddpar.h"

globals::globals()
	//: cRoot(nullptr)
{
	BOOST_TEST_MESSAGE( "Loading globals" );
	instance() = this; // Really only meant to be invoked once
	//rtmath::debug::instances::registerInstance( "test globals", reinterpret_cast<void*>(this));

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
		set<string> cand;
		cand.insert("./data");
		cand.insert("../rtmath/rtmath/tests/data");

		bool done = false;
		for (auto it = cand.begin(); it != cand.end() && done == false; it++)
		{
			pTestData = path(*it);
			if (exists(pTestData)) done = true;
		}
		if (!done)
			BOOST_FAIL("Cannot find test data directory.");
	}
	
	// Find full data
	{
		set<string> cand;
		cand.insert("../data");
		cand.insert("../rtmath/rtmath/data");
		bool done = false;
		for (auto it = cand.begin(); it != cand.end() && done == false; it++)
		{
			pFullData = path(*it);
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
	try {
		if (!exists(pRtconf))
			throw rtmath::debug::xMissingFile(pRtconf.string().c_str());
		cRoot =  rtmath::config::loadRtconfRoot(pRtconf.string());
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		BOOST_FAIL("Cannot load testing rtmath.conf file! Many tests cannot proceed.");
	}
}



globals*& globals::instance()
{
	static globals* s_inst = nullptr;
	return s_inst;
}



ddparloader::ddparloader()
	: defaultKey(false),
	baseExists(false),
	testPar(nullptr)
{
	using namespace std;
	using namespace boost::filesystem;
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
		//testPar->_populateDefaults();
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		BOOST_FAIL("Unable to load default testing ddscat.par file. Cannot do ddPar tests.");
	}
}

ddOutputSingleLoader::ddOutputSingleLoader()
{
	using namespace std;
	using namespace boost::filesystem;
	string sAvg1, sAvg2, sSca, sFml, sXml, sXmlbz2;
	globals::instance()->cRoot->getVal<string>("ddOutputSingle/Avg1", sAvg1);
	globals::instance()->cRoot->getVal<string>("ddOutputSingle/Avg2", sAvg2);
	globals::instance()->cRoot->getVal<string>("ddOutputSingle/Sca", sSca);
	globals::instance()->cRoot->getVal<string>("ddOutputSingle/Fml", sFml);
	globals::instance()->cRoot->getVal<string>("ddOutputSingle/Xml1", sXml);
	globals::instance()->cRoot->getVal<string>("ddOutputSingle/Xml2", sXmlbz2);

	pAvg1 = boost::filesystem::path(sAvg1);
	pAvg2 = boost::filesystem::path(sAvg2);
	pSca = boost::filesystem::path(sSca);
	pFml = boost::filesystem::path(sFml);
	pXml = boost::filesystem::path(sXml);
	pXmlbz2 = boost::filesystem::path(sXmlbz2);
}

ddparloader::~ddparloader()
{
}

ddOutputSingleLoader::~ddOutputSingleLoader() {}
