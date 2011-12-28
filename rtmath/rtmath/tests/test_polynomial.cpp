#include <string>
#include <iostream>
#include "../polynomial.h"

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(test_polynomial);

// Check set/get basic assignment
BOOST_AUTO_TEST_CASE(polynomial_assign) {
	rtmath::polynomial asq(2,2.0);
	std::cout << asq << std::endl;
	BOOST_CHECK_EQUAL(asq.coeff(2),2.0);
}


BOOST_AUTO_TEST_SUITE_END();

