#include <string>
#include <iostream>
#define BOOST_TEST_DYN_LINK
#include "globals.h"
#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <memory>

#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/ddscat/hulls.h"
#include "../rtmath/ddscat/crossSection.h"
#include "../rtmath/ddscat/shapefile.h"
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
		auto shp = rtmath::ddscat::shapefile(
			(globals::instance()->pTestData / "miniflakehole2db.shp").string());
		boost::shared_ptr<shapeFileStats> stats(new shapeFileStats(shp));
		shapeFileStatsDipoleView baseView(stats, 10.0);

		auto statsRot0 = stats->calcStatsRot(0,0,0);
		shapeFileStatsRotatedView rotView(statsRot0, 10.0);

		concaveHull hull(shp._latticePtsStd);
		hull.constructHull(1.01);
		hull.writeVTKhull("outhull.vtk");
		writeVTKpoints("outhullpts.vtk", hull._hullPts);
		writeVTKpoints("outpts.vtk", hull._points);

		crossSection crs(shp._latticePtsStd);
		crs.construct(1, 0,0,1);
		writeVTKpoints("outPtsZ.vtk", crs._projpoints);
		crs.writeVTKhull("outPolysZ.vtk");
		crs.construct(1, 0,1,0);
		writeVTKpoints("outPtsY.vtk", crs._projpoints);
		crs.writeVTKhull("outPolysY.vtk");
		crs.construct(1, 1,0,0);
		writeVTKpoints("outPtsX.vtk", crs._projpoints);
		crs.writeVTKhull("outPolysX.vtk");
		crs.construct(1, 1,0,1);
		writeVTKpoints("outPtsXZ.vtk", crs._projpoints);
		crs.writeVTKhull("outPolysXZ.vtk");

		cout << "Test" << endl;
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		BOOST_FAIL("Exception thrown.");
	}
}


BOOST_AUTO_TEST_SUITE_END();

