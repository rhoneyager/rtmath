#include <string>
#include <iostream>
#include <boost/test/unit_test.hpp>

#define _DEBUG
#include "../debug.h"
//#define BOOST_TEST_MODULE matrixop
//#define BOOST_TEST_DYN_LINK


BOOST_AUTO_TEST_SUITE(test_debug);

//using namespace rtmath;
/*
// Check set/get alignment
BOOST_AUTO_TEST_CASE(matrixop_set) {
	rtmath::matrixop a(2,4,4);
	a.set(3.0,2,0,0);
	a.set(4.0,2,3,2);
	BOOST_CHECK_EQUAL(a.get(2,0,0),3.0);
	BOOST_CHECK_EQUAL(a.get(2,3,2),4.0);
}
*/

BOOST_AUTO_TEST_CASE(debug_preamble) {
	rtmath::debug::debug_preamble();
}

BOOST_AUTO_TEST_CASE(debug_revision_flag)
{
	int rev = rtmath::debug::rev();
	bool revision_flag = false;
	if (rev != -1) revision_flag = true;
	BOOST_CHECK_EQUAL(revision_flag,true);
}

BOOST_AUTO_TEST_CASE(debug_setloc)
{
	// Construct a throw. If setloc provides information, 
	// test passes. If not, test failed.
	try {
		throw rtmath::debug::xUnimplementedFunction();
	}
	catch (rtmath::debug::xError &err)
	{
		//bool res = err.hasLoc();
		//err.Display();
		BOOST_CHECK_EQUAL(err.hasLoc(), true);
	}
}

BOOST_AUTO_TEST_CASE(debug_class_markers) {
}

BOOST_AUTO_TEST_CASE(debug_general_markers) {
}

BOOST_AUTO_TEST_CASE(debug_marks) {
}
BOOST_AUTO_TEST_SUITE_END();

