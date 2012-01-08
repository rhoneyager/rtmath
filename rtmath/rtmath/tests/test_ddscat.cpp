#include <string>
#include <iostream>
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>
#include "../rtmath/ddscat.h"

BOOST_AUTO_TEST_SUITE(test_ddscat);

// Might as well set these, as it would appear in every 
// function anyways.
using namespace rtmath;
using namespace rtmath::ddscat;

// Not going to check file reads for now, as I 
// would need to include a test data file set

// ddScattMatrix-inspired tests
BOOST_AUTO_TEST_CASE(ddScattMatrix_init)
{
}

BOOST_AUTO_TEST_CASE(ddScattMatrix_accessorFuncs)
{
}

BOOST_AUTO_TEST_CASE(ddScattMatrix_equality)
{
}

BOOST_AUTO_TEST_CASE(ddScattMatrix_addition)
{
}

BOOST_AUTO_TEST_CASE(ddScattMatrix_muellerCalc)
{
}

BOOST_AUTO_TEST_CASE(ddScattMatrix_extinctionCalc)
{
}

BOOST_AUTO_TEST_CASE(ddScattMatrix_Scalc)
{
}

BOOST_AUTO_TEST_CASE(ddCoordsChecks)
{
	ddCoords a(3.2,2.8), b(1.1,2.2), c(3.2,2.8);
	ddCoords3 d(1.6,7.2,8.9), e(1.6,7.2,8.9), f(1,2,3);
	// Verifying assignment
	BOOST_CHECK(a.theta == 3.2);
	BOOST_CHECK(b.phi == 2.2);
	BOOST_CHECK(c.alpha == a.theta);
	BOOST_CHECK(d.beta == 1.6);
	BOOST_CHECK(f.phi == 3);
	BOOST_CHECK(e.theta == d.theta);
	// Checking both types of comparison here
	BOOST_CHECK(a!=b);
	BOOST_CHECK(a==c);
	BOOST_CHECK(b!=c);
	BOOST_CHECK(d==e);
	BOOST_CHECK(d!=f);
	BOOST_CHECK(e!=f);
	ddCoordsComp cmp;
	BOOST_CHECK(cmp.operator()(a,b) == false);
	BOOST_CHECK(cmp.operator()(a,c) == true);
	BOOST_CHECK(cmp.operator()(c,b) == false);
	BOOST_CHECK(cmp.operator()(d,e) == true);
	BOOST_CHECK(cmp.operator()(d,f) == false);
	BOOST_CHECK(cmp.operator()(e,f) == false);
}


// ddOutputSingle-inspired tests

BOOST_AUTO_TEST_CASE(ddOutputSingle_construction)
{
}

BOOST_AUTO_TEST_CASE(ddOutputSingle_f)
{
}

BOOST_AUTO_TEST_CASE(ddOutputSingle_eval)
{
}

BOOST_AUTO_TEST_CASE(ddOutputSingle_size)
{
}


// ddOutputEnsemble testing

// Isotropic ensemble verification

// Gaussian ensembles


// ddOutput testing
// Setting and getting
// Loading



BOOST_AUTO_TEST_SUITE_END();

