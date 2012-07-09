#include <string>
#include <iostream>
#define BOOST_TEST_DYN_LINK
#include "globals.h"
#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <memory>

#include "../rtmath/rtmath.h"


BOOST_AUTO_TEST_SUITE(test_ddparGenerator);

using namespace rtmath;
using namespace std;
using namespace boost;
using namespace rtmath::debug;
using namespace rtmath::ddscat;
using namespace rtmath::units;

// Test to construct ddparGenerator
BOOST_AUTO_TEST_CASE(pargenerator_construct)
{
	// Trivial construction
	ddParGenerator p;
	// Known base ddPar
	ddParGenerator q(*rtmath::ddscat::ddPar::defaultInstance());
}

// Test to prepare a sample ddparGenerator file
BOOST_AUTO_TEST_CASE(pargenerator_sample)
{
	ddParGenerator p;
	p.name = "test run";
	p.description = "desc goes here";
	p.outLocation = "./testrun";

	p.ddscatVer = 72;
	p.compressResults = true;

}

// Read from ddPar file




// Read a known ddparGenerator file

// Attempt to generate a batch of runs

//

BOOST_AUTO_TEST_SUITE_END();

