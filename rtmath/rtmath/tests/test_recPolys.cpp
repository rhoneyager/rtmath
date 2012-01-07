#include <string>
#include <iostream>
#include "../rtmath/polynomial.h"
#include "../rtmath/polynomials/recursivePolynomial.h"
#include "../rtmath/polynomials/legendre.h"
#include "../rtmath/polynomials/laguerre.h"
#include "../rtmath/polynomials/chebyshev.h"
#include "../rtmath/polynomials/hermite.h"

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(test_recursive_polynomials);

BOOST_AUTO_TEST_CASE(legendre) {
	// Generate and compare against precalculated polynomials
	using namespace rtmath;
	using namespace rtmath::recPolys;
	polynomial x(1,1.0);
	polynomial l2 = ( (x^2)*3.0 - 1.0 ) * 0.5;
	polynomial l5 = ( (x^5)*63.0 - (x^3)*70.0 + x*15.0) * 0.125;
	polynomial l8 = ( (x^8)*6435 - (x^6)*12012 + (x^4) * 6930 - (x^2) *1260 + 35.0) * (1.0/128.0);
	polynomial l10 = ( (x^10)*46189 - (x^8)*109395 + (x^6)*90090 - (x^4)*30030 + (x^2)*3465 - 63.0 ) * (1.0/256.0);
	polynomial g2,g5,g8,g10;
	rtmath::recPolys::legendre legGen;
	legGen.get(2,g2);
	legGen.get(8,g8);
	legGen.get(5,g5);
	legGen.get(10,g10);
	// Note: I can't just test for equality directly, as the calculation will be off by a small amount (like 1e-18)
	// for some coefficients, which will cause the compile to fail. Instead, use the other polynomial operator (~=)
	// to determine approximate equality (to with 0.0001%)
	BOOST_CHECK(l2.approxEq(g2));
	BOOST_CHECK(l5.approxEq(g5));
	BOOST_CHECK(l8.approxEq(g8));
	BOOST_CHECK(l10.approxEq(g10));
}

BOOST_AUTO_TEST_CASE(laguerre) {
	using namespace rtmath;
	using namespace rtmath::recPolys;
	polynomial x(1,1.0);
	polynomial l2 = ((x^2)-x*4+2)*0.5;
	polynomial l3 = ((x^3)*-1 + (x^2)*9 - x*18 + 6) * (1./6.);
	polynomial l6 = ((x^6) - (x^5)*36 + (x^4)*450 - (x^3)*2400 + (x^2)*5400 - x*4320 + 720) * (1./720.);
	polynomial g2,g3,g6;
	rtmath::recPolys::laguerre lagGen;
	lagGen.get(2,g2);
	lagGen.get(6,g6);
	lagGen.get(3,g3);
	BOOST_CHECK(l2.approxEq(g2));
	BOOST_CHECK(l3.approxEq(g3));
	BOOST_CHECK(l6.approxEq(g6));
}

BOOST_AUTO_TEST_CASE(chebyshev_a) {
	using namespace rtmath;
	using namespace rtmath::recPolys;
	polynomial x(1,1.0);
	polynomial l3 = (x^3)*4 - x*3;
	polynomial l6 = (x^6)*32 - (x^4)*48 + (x^2)*18 - 1;
	polynomial l9 = (x^9)*256 - (x^7)*576 + (x^5)*432 - (x^3)*120 + x*9;
	polynomial g3,g6,g9;
	rtmath::recPolys::chebyshevA gen;
	gen.get(3,g3);
	gen.get(9,g9);
	gen.get(6,g6);
	BOOST_CHECK(l3.approxEq(g3));
	BOOST_CHECK(l6.approxEq(g6));
	BOOST_CHECK(l9.approxEq(g9));
}

BOOST_AUTO_TEST_CASE(chebyshev_b) {
	using namespace rtmath;
	using namespace rtmath::recPolys;
	polynomial x(1,1.0);
	polynomial l3 = (x^3)*8 - x*4;
	polynomial l6 = (x^6)*64 - (x^4)*80 + (x^2)*24 - 1;
	polynomial l9 = (x^9)*512 - (x^7)*1024 + (x^5)*672 - (x^3)*160 + x*10;
	polynomial g3,g6,g9;
	rtmath::recPolys::chebyshevB gen;
	gen.get(3,g3);
	gen.get(9,g9);
	gen.get(6,g6);
	BOOST_CHECK(l3.approxEq(g3));
	BOOST_CHECK(l6.approxEq(g6));
	BOOST_CHECK(l9.approxEq(g9));
}

BOOST_AUTO_TEST_CASE(hermite) {
	using namespace rtmath;
	using namespace rtmath::recPolys;
	polynomial x(1,1.0);
	polynomial l3 = (x^3) - x*3;
	polynomial l6 = (x^6) - (x^4)*15 + (x^2)*45 - 15;
	polynomial l9 = (x^9) - (x^7)*36 + (x^5)*378 - (x^3)*1260 + x*945;
	polynomial g3,g6,g9;
	rtmath::recPolys::hermite gen;
	gen.get(3,g3);
	gen.get(9,g9);
	gen.get(6,g6);
	BOOST_CHECK(l3.approxEq(g3));
	BOOST_CHECK(l6.approxEq(g6));
	BOOST_CHECK(l9.approxEq(g9));
}

BOOST_AUTO_TEST_CASE(hermite_phys) {
	using namespace rtmath;
	using namespace rtmath::recPolys;
	polynomial x(1,1.0);
	polynomial l3 = (x^3)*8 - x*12;
	polynomial l6 = (x^6)*64 - (x^4)*480 + (x^2)*720 - 120;
	polynomial l9 = (x^9)*512 - (x^7)*9216 + (x^5)*48384 - (x^3)*80640 + x*30240;
	polynomial g3,g6,g9;
	rtmath::recPolys::hermitePhys gen;
	gen.get(3,g3);
	gen.get(9,g9);
	gen.get(6,g6);
	BOOST_CHECK(l3.approxEq(g3));
	BOOST_CHECK(l6.approxEq(g6));
	BOOST_CHECK(l9.approxEq(g9));
}

/*
BOOST_AUTO_TEST_CASE(associated_legendre) {
}
*/

BOOST_AUTO_TEST_SUITE_END();

