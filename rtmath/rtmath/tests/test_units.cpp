#include <string>
#include <iostream>
#define BOOST_TEST_DYN_LINK
#include "../rtmath/units.h"

#include <boost/test/unit_test.hpp>
#include "../rtmath/error/error.h"

BOOST_AUTO_TEST_SUITE(test_units);

using namespace std;
using namespace rtmath;
using namespace rtmath::units;

// Construct atmoslayer
BOOST_AUTO_TEST_CASE(units_spec) {
	conv_spec fwv("GHz","m");
	conv_spec wvf("m","GHz");
	conv_spec fwn("GHz","m^-1");
	conv_spec fwn2("GHz","cm^-1");
	conv_spec wnf("m^-1","GHz");
	conv_spec wnf2("cm^-1","Hz");

	// Check conversion from GHz to m
	double in = 0.1;
	double out = fwv.convert(in);
	BOOST_CHECK_CLOSE(out,2.9979,0.01);
	// Check reverse conversion
	in = wvf.convert(out);
	BOOST_CHECK_CLOSE(in,0.1,0.001);
	// Check conversion to m^-1
	in = 0.1;
	out = fwn.convert(in);
	BOOST_CHECK_CLOSE(out,0.33356,0.01);
	in = 0.1;
	out = fwn2.convert(in);
	BOOST_CHECK_CLOSE(out,0.0033356,0.01);
	in = 0.0033356;
	out = wnf2.convert(in);
	BOOST_CHECK_CLOSE(out,1.e8,0.01);
	in = 0.33356;
	out = wnf.convert(in);
	BOOST_CHECK_CLOSE(out,0.1,0.01);

	conv_spec imcm("cm^-1","m^-1");
	in = 0.0033356;
	out = imcm.convert(in);
	BOOST_CHECK_CLOSE(out,0.33356,0.01);
}

BOOST_AUTO_TEST_CASE(units_pres) {
	conv_pres a("mb", "hPa");
	BOOST_CHECK_EQUAL(15,a.convert(15)); // Should be an identity
	conv_pres b("hPa", "Pa");
	BOOST_CHECK_EQUAL(b.convert(15),1500);
	conv_pres c("Pa", "hPa");
	BOOST_CHECK_EQUAL(c.convert(1500),15);
	conv_pres d("kPa", "hPa");
	BOOST_CHECK_EQUAL(d.convert(15),150);
	conv_pres e("bar", "hPa");
	BOOST_CHECK_EQUAL(e.convert(2.5),2500);
}

BOOST_AUTO_TEST_CASE(units_alt) {
	BOOST_CHECK_CLOSE(100,conv_alt("m","cm").convert(1),0.01);
	BOOST_CHECK_CLOSE(1,conv_alt("cm","m").convert(100),0.01);
	BOOST_CHECK_CLOSE(100,conv_alt("km","m").convert(0.1),0.01);
	BOOST_CHECK_CLOSE(3.4,conv_alt("cm","km").convert(340000),0.01);
}

BOOST_AUTO_TEST_CASE(units_dens) {
	BOOST_CHECK_CLOSE(1e-6,conv_dens("m^-3","cm^-3").convert(1),0.01);
	BOOST_CHECK_CLOSE(3.5,conv_dens("cm^-3","m^-3").convert(3.5e-6),0.01);
}

BOOST_AUTO_TEST_SUITE_END();

