#include <string>
#include <iostream>
#define BOOST_TEST_DYN_LINK
#include "globals.h"
#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <memory>

#include "../rtmath/serialization.h"

using namespace rtmath;
using namespace rtmath::ddscat;
using namespace std;
using namespace rtmath::config;
using namespace boost::filesystem;


BOOST_AUTO_TEST_SUITE(test_serialization);
//BOOST_FIXTURE_TEST_SUITE(test_serialization, ddparloader);
// globals::instance()->

// Test that ddscat.par file can load
BOOST_AUTO_TEST_CASE(serialization_basic)
{
}

BOOST_AUTO_TEST_CASE(serialization_compression_list)
{
}

BOOST_AUTO_TEST_CASE(serialization_compression_gzip)
{
}

BOOST_AUTO_TEST_CASE(serialization_compression_zlib)
{
}

BOOST_AUTO_TEST_CASE(serialization_compression_bzip2)
{
}

BOOST_AUTO_TEST_SUITE_END();

