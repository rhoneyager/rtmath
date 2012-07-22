#include <string>
#include <iostream>
#define BOOST_TEST_DYN_LINK
#include "globals.h"
#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <memory>

#include "../rtmath/rtmath.h"


BOOST_AUTO_TEST_SUITE(test_shapefile);
using namespace rtmath;
using namespace std;
using namespace boost;
using namespace rtmath::debug;
using namespace rtmath::ddscat;
using namespace rtmath::units;

// Test shapefile read and write
BOOST_AUTO_TEST_CASE(shapefile_io)
{
	shapefile shp((globals::instance()->pTestData / "2mm12shape.txt").string());
	
	ostringstream out, outb;
	shp.write(out);
	string str = out.str(), strb;
	istringstream in(str);
	shapefile sb(in);
	sb.write(outb);
	strb = outb.str();
	BOOST_CHECK(str == strb);
	//ofstream outaf("outa.shp"), outbf("outb.shp");
	//outaf << str;
	//outbf << strb;
	
}

// Test shapefile stats
BOOST_AUTO_TEST_CASE(shapefile_stats)
{
	shapefile shp((globals::instance()->pTestData / "2mm12shape.txt").string());
	shapeFileStats sshp(shp);
	BOOST_TEST_MESSAGE("Base");
	sshp.calcStatsBase();
	BOOST_TEST_MESSAGE("0 0 0");
	sshp.calcStatsRot(0,0,0);
	BOOST_TEST_MESSAGE("0 30 0");
	sshp.calcStatsRot(0,30,0);
	BOOST_TEST_MESSAGE("0 0 30");
	sshp.calcStatsRot(0,0,30);
	BOOST_TEST_MESSAGE("0 30 30");
	sshp.calcStatsRot(0,30,30);

	BOOST_TEST_MESSAGE("Writing");
	rtmath::serialization::write<shapeFileStats>(sshp,"shpstats.xml");
}


BOOST_AUTO_TEST_SUITE_END();

