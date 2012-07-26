#include <string>
#include <iostream>
#define BOOST_TEST_DYN_LINK
#include "globals.h"
#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <memory>
#include <set>

#include "../rtmath/rtmath.h"
#include "../rtmath/ddscat/shapestats.h"
#include "../rtmath/ddscat/hulls.h"
#include "../rtmath/PCLlink.h"

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
	shapefile shp((globals::instance()->pTestData / "miniflake.shp").string());
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
	BOOST_TEST_MESSAGE("30 0 0");
	sshp.calcStatsRot(30,0,0);
	BOOST_TEST_MESSAGE("30 30 30");
	sshp.calcStatsRot(30,30,30);

	BOOST_TEST_MESSAGE("Writing");
	rtmath::serialization::write<shapeFileStats>(sshp,"shpstats.xml");
}

BOOST_AUTO_TEST_CASE(shapefile_vtk)
{
	shapefile shp((globals::instance()->pTestData / "ddscat-snow-33-23-23.shp").string());
	//shapeFileStats sshp(shp);
	writeVTKpoints("miniflake-points.vtk", shp._latticePtsStd);
	hull h(shp._latticePtsStd);
	h.searchRadius = 2.0;
	h.Mu = 2.0;
	h.writeVTKhull("miniflake-hull.vtk");
	convexHull cvx(shp._latticePtsStd);
	cvx.constructHull();
	cout << "Max diameter is: " << cvx.maxDiameter() << endl;
	cvx.searchRadius = 6.0;
	cvx.Mu = 6.0;
	cvx.writeVTKhull("miniflake-convex-hull.vtk");
	writeVTKpoints("miniflake-convex-points.vtk", cvx._hullPts);

	concaveHull ccv(shp._latticePtsStd);
	double myalphas[] = { 1.0, 2.0, 4.0, 8.0 };
	set<double> alphas(myalphas, myalphas+4);
	for (auto it = alphas.begin(); it != alphas.end(); it++)
	{
		ccv.constructHull(*it);
		if (ccv._hullPts.size() == 0) continue;
		ostringstream fpts, fhull;
		fpts << "miniflake-concave-" << *it << "-pts.vtk";
		fhull << "miniflake-concave-" << *it << "-hull.vtk";
		ccv.writeVTKhull(fhull.str());
		writeVTKpoints(fpts.str(), ccv._hullPts);
	}
}

BOOST_AUTO_TEST_SUITE_END();

