#include <string>
#include <iostream>
#include "../polynomial.h"

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(test_polynomial);

// Check set/get basic assignment
BOOST_AUTO_TEST_CASE(polynomial_assign) {
	rtmath::polynomial asq(2,2.0);
	BOOST_CHECK_EQUAL(asq.coeff(2),2.0);
}

BOOST_AUTO_TEST_CASE(polynomial_evaluation) {
	using namespace rtmath;
	polynomial x(1,1.0);
	polynomial y = ((x^2) + (x * 2.0)) + 1.0;
	double res = y.eval(-3.0);
	BOOST_CHECK_EQUAL(4.0,res);
	res = y(-1.0);
	BOOST_CHECK_EQUAL(res,0.0);
	res = y[1];
	BOOST_CHECK_EQUAL(res,2.0);
}

BOOST_AUTO_TEST_CASE(polynomial_equalities) {
	using namespace rtmath;
	polynomial x(1,1.0);
	polynomial y(1,2.0);
	polynomial z(2,1.0);
	bool eq = false;
	eq = x==y;
	BOOST_CHECK_EQUAL(eq,false);
	eq = x==z;
	BOOST_CHECK_EQUAL(eq,false);
	eq = y==y;
	BOOST_CHECK_EQUAL(eq,true);
}

BOOST_AUTO_TEST_CASE(polynomial_addition) {
	using namespace rtmath;
	polynomial x(1,1.0);
	polynomial a = (x^2) + x*2.0 + 1.0;
	polynomial b = (x^2) + 1.0;
	polynomial c = x*2.0;
	polynomial d = b + c;
	bool eq = false;
	if (a == d) eq = true;
	BOOST_CHECK_EQUAL(eq,true);
}

BOOST_AUTO_TEST_CASE(polynomial_subtraction) {
	using namespace rtmath;
	polynomial x(1,1.0);
	polynomial a = (x^3) + (x^2) * 2.0 + x * 3.0 - 1.0;
	polynomial b = (x^2) - 1.0;
	polynomial c = (x^3) + (x^2) + x * 3.0;
	bool eq = false;
	if (a-b==c) eq = true;
	BOOST_CHECK_EQUAL(eq,true);
}

BOOST_AUTO_TEST_CASE(polynomial_multiplication) {
	using namespace rtmath;
	polynomial x(1,1.0);
	polynomial a = (x^2) + x*2.0 + 1.0;
	polynomial b = x - 1.0;
	polynomial c = (x^3) + (x^2) - x - 1.0;
	bool eq = false;
	if (c==a*b) eq = true;
	BOOST_CHECK_EQUAL(eq,true);
}

BOOST_AUTO_TEST_CASE(polynomial_exponentiation) {
	using namespace rtmath;
	polynomial x(1,1.0);
	polynomial a = (x^2) - 1.0;
	polynomial b = a^2;
	polynomial c = (x^4) - (x^2) * 2.0 + 1.0;
	bool eq = false;
	if (c==b) eq = true;
	BOOST_CHECK_EQUAL(eq,true);
}

BOOST_AUTO_TEST_CASE(polynomial_erasure) {
	using namespace rtmath;
	polynomial x(1,1.0);
	x.erase();
	polynomial y(0,0.0);
	bool res = (x==y);
	BOOST_CHECK_EQUAL(res,true);
}

BOOST_AUTO_TEST_CASE(polynomial_truncation) {
	using namespace rtmath;
	polynomial x(1,1.0);
	polynomial y = (x^4) + (x^3) - (x^2) - x + 1.0;
	polynomial z = (x^2) * -1.0 - x + 1.0;
	polynomial h = y;
	h.truncate(2);
	bool res = (h == z);
	BOOST_CHECK_EQUAL(res,true);
}

BOOST_AUTO_TEST_CASE(polynomial_derivative) {
	using namespace rtmath;
	polynomial x(1,1.0);
	polynomial a = (x^4) * 3.0 + (x^3) * 2.0 - x + 1.0;
	polynomial b = a.deriv(1);
	polynomial c = (x^3) * 12.0 + (x^2) * 6.0 - 1.0;
	bool res = false;
	if (c==b) res = true;
	BOOST_CHECK_EQUAL(res,true);
}

//BOOST_AUTO_TEST_CASE(polynomial_integration) {
//}

//BOOST_AUTO_TEST_CASE(polynomial_zeros) {
//}

BOOST_AUTO_TEST_SUITE_END();

