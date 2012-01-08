#include <string>
#include <iostream>
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

//#define _DEBUG
#include "../rtmath/rtmath.h"
//#define BOOST_TEST_MODULE matrixop
//#define BOOST_TEST_DYN_LINK


BOOST_AUTO_TEST_SUITE(test_debug);


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


