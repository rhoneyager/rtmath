#include <string>
#include <iostream>
#define BOOST_TEST_DYN_LINK
#include "globals.h"
#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <memory>

#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/ddscat/shapestats.h"
#include "../rtmath/ddscat/shapestatsRotated.h"
#include "../rtmath/ddscat/shapestatsviews.h"
#include "../rtmath/Serialization/shapestats_serialization.h"
#include "../rtmath/serialization.h"

using namespace rtmath;
using namespace rtmath::ddscat;
using namespace std;
using namespace rtmath::config;
using namespace boost::filesystem;

/* Notes:
 * no facility for testing / comparing versioned reads and writes
 */

//BOOST_AUTO_TEST_SUITE(test_ddpar);
BOOST_AUTO_TEST_SUITE(test_shapestats);
// globals::instance()->
using namespace rtmath;
using namespace std;
using namespace boost;
using namespace rtmath::debug;
using namespace rtmath::ddscat;

// Test that basic stats can be generated
BOOST_AUTO_TEST_CASE(shapefile_stats)
{
	try {
		rtmath::ddscat::shapeFileStats::doQhull(false);
		auto stats = rtmath::ddscat::shapeFileStats::genStats(
			(globals::instance()->pTestData / "miniflake.shp").string());
		shapeFileStatsDipoleView baseView(stats, 10.0);

		auto statsRot0 = stats->calcStatsRot(0,0,0);
		shapeFileStatsRotatedView rotView(statsRot0, 10.0);

		cout << "Test" << endl;
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		BOOST_FAIL("Exception thrown.");
	}
}


BOOST_AUTO_TEST_SUITE_END();

