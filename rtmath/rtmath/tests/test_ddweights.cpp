#include <string>
#include <iostream>
#include <memory>
#include <set>
#include <map>
#define BOOST_TEST_DYN_LINK
//#include "../rtmath/ddscat/ddweights.h"
//#include "../rtmath/ddscat/ddscat.h"
#include "../rtmath/error/error.h"

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(test_ddweights);

using namespace std;
using namespace rtmath;
using namespace rtmath::debug;
//using namespace rtmath::ddscat;

BOOST_AUTO_TEST_CASE(gPosWeights)
{
	/*
	// Begin test by creating a set of points
	set<double> points;
	for (double i=0.5; i<90; i++)
	{
		points.insert(i);
	}

	gaussianPosWeights dist1(1.0,points), dist2(2.0,points), 
		dist5(5.0,points), dist10(10.0,points), dist30(30.0,points);

	/*
	cout << "Point\t1.010.0\t30.0\n";
	set<double>::const_iterator it;
	for (it = points.begin(); it != points.end(); it++)
	{
		cout << *it << "\t" << dist1.weight(*it) << "\t" << dist10.weight(*it)
			<< "\t" << dist30.weight(*it) << endl;
	}

	cout << endl << endl;
	*/
}




BOOST_AUTO_TEST_SUITE_END();

