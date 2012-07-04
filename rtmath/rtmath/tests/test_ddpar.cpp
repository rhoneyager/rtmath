#include <string>
#include <iostream>
#define BOOST_TEST_DYN_LINK
#include "globals.h"
#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <memory>

#include "../rtmath/rtmath.h"


BOOST_AUTO_TEST_SUITE(test_ddpar);

using namespace rtmath;
using namespace std;
using namespace rtmath::config;
using namespace boost::filesystem;
// globals::instance()->

// TODO: use a better set of scoped function handlers

BOOST_AUTO_TEST_CASE(ddpar_defaultExists)
{
	string sBasePar, scwd;
	bool defaultKey = false;
	defaultKey = globals::instance()->cRoot->getVal<string>("ddscat/DefaultFile", sBasePar);
	globals::instance()->cRoot->getCWD(scwd);
	if (!defaultKey) BOOST_FAIL("ddscat/DefaultFile key missing from testing rtmath.conf");
	path p(sBasePar);
	p = boost::filesystem::absolute(p,path(scwd).remove_filename());
	if (!exists(p)) BOOST_FAIL("ddscat/DefaultFile key refers to nonexistant file");
}

BOOST_AUTO_TEST_CASE(ddpar_defaultLoad)
{
	string sBasePar, scwd;
	globals::instance()->cRoot->getCWD(scwd);
	path pscwd(scwd);
	pscwd.remove_filename();

	bool defaultKey = globals::instance()->cRoot->getVal<string>("ddscat/DefaultFile", sBasePar);

	path pBasePar(sBasePar);
	if (pBasePar.is_relative()) pBasePar = pscwd / pBasePar;

	BOOST_REQUIRE(defaultKey==true);
	rtmath::ddscat::ddPar par(pBasePar.string());
	// If errors encountered, this case will fail. If no exceptions, then it succeded
}



BOOST_AUTO_TEST_SUITE_END();

