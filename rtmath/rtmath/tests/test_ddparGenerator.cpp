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
}

// Test to prepare a sample ddparGenerator file

// Read a known ddparGenerator file

// Attempt to generate a batch of runs

//

BOOST_AUTO_TEST_SUITE_END();

