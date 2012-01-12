#include <string>
#include <iostream>
#define BOOST_TEST_DYN_LINK
#include "../rtmath/rtmath.h"

#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <memory>

BOOST_AUTO_TEST_SUITE(test_config);

using namespace rtmath;
using namespace std;
using namespace rtmath::config;

configsegment* cRoot;

void loadConfig()
{
	string crootname = "tests/data/rtmath.conf";
	cRoot =  configsegment::loadFile(
			crootname.c_str(),NULL);
}

BOOST_AUTO_TEST_CASE(config_loadFile)
{
	try {
		loadConfig();
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		BOOST_FAIL("Test threw an error.");
	}
}

BOOST_AUTO_TEST_CASE(config_getVal)
{
	//shared_ptr<configsegment> cRoot;
	//loadConfig(cRoot);
	string erfile;
	cRoot->getVal("maxcores", erfile);
	BOOST_CHECK(erfile.size() > 0);
	cRoot->getVal("testing/wvlow", erfile);
	BOOST_CHECK(erfile.size() > 0);
}

BOOST_AUTO_TEST_CASE(config_setVal)
{
	const string test = "hi";
	string check;
	cRoot->setVal("hiya", test);
	cRoot->getVal("hiya", check);
	BOOST_CHECK_EQUAL(test,check);
}

BOOST_AUTO_TEST_CASE(config_findSegment)
{
	configsegment *ta = cRoot->findSegment("testing");
	BOOST_REQUIRE_MESSAGE(ta!=cRoot, "findSegment is not working right. FIX IT.");
	BOOST_REQUIRE(ta!=0);
	BOOST_CHECK(ta != 0);
	configsegment *tb = cRoot->getChild("testing");
	string wl;
	ta->getVal("wvlow", wl);
	BOOST_CHECK(wl.size() > 0);
	tb->getVal("wvlow", wl);
	BOOST_CHECK(wl.size() > 0);
	BOOST_CHECK(ta == tb);
}

BOOST_AUTO_TEST_CASE(config_getChild)
{
	configsegment *tb = cRoot->getChild("testing");
	string wl;
	tb->getVal("wvlow",wl);
	BOOST_CHECK(wl.size()>0);
}

BOOST_AUTO_TEST_CASE(config_getParent)
{
	configsegment *tb = cRoot->getChild("testing");
	configsegment *par = tb->getParent();
	BOOST_CHECK(cRoot==par);
	par = cRoot->getParent();
	BOOST_CHECK(par == NULL);
}

BOOST_AUTO_TEST_CASE(config_finished)
{
	delete cRoot;
}

// queryConfig not tested, as user interaction may be required

BOOST_AUTO_TEST_CASE(config_defaultroot)
{
	std::string f;
	getConfigDefaultFile(f);
	BOOST_CHECK(f != "");
	BOOST_REQUIRE_MESSAGE(f!="", "The makefile is not providing a default rtmath config file path. FIX IT.");
}

BOOST_AUTO_TEST_SUITE_END();

