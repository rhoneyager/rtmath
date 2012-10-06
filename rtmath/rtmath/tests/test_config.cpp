#include <string>
#include <iostream>
#define BOOST_TEST_DYN_LINK
#include "globals.h"
#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <memory>

#include "../rtmath/error/error.h"
#include "../rtmath/config.h"

BOOST_AUTO_TEST_SUITE(test_config);

using namespace rtmath;
using namespace std;
using namespace rtmath::config;

BOOST_AUTO_TEST_CASE(config_getVal)
{
	//shared_ptr<configsegment> cRoot;
	//loadConfig(cRoot);
	string erfile;
	globals::instance()->cRoot->getVal("General/maxcores", erfile);
	BOOST_CHECK(erfile.size() > 0);
	globals::instance()->cRoot->getVal("testing/wvlow", erfile);
	BOOST_CHECK(erfile.size() > 0);
}

BOOST_AUTO_TEST_CASE(config_setVal)
{
	const string test = "hi";
	string check;
	globals::instance()->cRoot->setVal("hiya", test);
	globals::instance()->cRoot->getVal("hiya", check);
	BOOST_CHECK_EQUAL(test,check);
}

BOOST_AUTO_TEST_CASE(config_findSegment)
{
	std::shared_ptr<configsegment> ta = globals::instance()->cRoot->findSegment("testing/");
	BOOST_REQUIRE_MESSAGE(ta!=globals::instance()->cRoot, "findSegment is not working right. FIX IT.");
	BOOST_REQUIRE(ta!=0);
	BOOST_CHECK(ta != 0);
	std::shared_ptr<configsegment> tb = globals::instance()->cRoot->getChild("testing");
	string wl;
	ta->getVal("wvlow", wl);
	BOOST_CHECK(wl.size() > 0);
	tb->getVal("wvlow", wl);
	BOOST_CHECK(wl.size() > 0);
	BOOST_CHECK(ta == tb);
}

BOOST_AUTO_TEST_CASE(config_getChild)
{
	std::shared_ptr<configsegment> tb = globals::instance()->cRoot->getChild("testing");
	string wl;
	tb->getVal("wvlow",wl);
	BOOST_CHECK(wl.size()>0);
}

BOOST_AUTO_TEST_CASE(config_getParent)
{
	std::shared_ptr<configsegment> tb = globals::instance()->cRoot->getChild("testing");
	std::shared_ptr<configsegment> par = tb->getParent();
	BOOST_CHECK(globals::instance()->cRoot==par);
	par = globals::instance()->cRoot->getParent();
	BOOST_CHECK(par == NULL);
}

// queryConfig not tested, as user interaction may be required

BOOST_AUTO_TEST_CASE(config_defaultroot)
{
	std::string f;
	getConfigDefaultFile(f);
	BOOST_CHECK(f != "");
	BOOST_REQUIRE_MESSAGE(f!="", "The build system is not providing a valid default rtmath config file path. FIX IT.");
}

BOOST_AUTO_TEST_SUITE_END();

