#include <string>
#include <iostream>
#include <memory>
#define BOOST_TEST_DYN_LINK
#include "../rtmath/atmos.h"
#include "../rtmath/atmoslayer.h"
#include "../rtmath/absorb.h"
#include "../rtmath/error/error.h"

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(test_atmos);

// These tests are designed to test the functionality of atmos and its associated classes.
// It will lead a sample atmosphere and perform basic optical depth calculations.
// To accomplish this, however, the functionality of the absorber derived classes and of 
// the atmoslayer class must also be verified.

using namespace rtmath;
using namespace rtmath::atmos;

// Construct atmoslayer
BOOST_AUTO_TEST_CASE(atmoslayer_initialization) {
	// atmoslayer comes first, as absorbers depend on it
	atmoslayer sample;
	sample.p(100); // Junk values for testing
	sample.T(200);
	sample.dz(110);

	BOOST_CHECK(sample.p() == 100);
	BOOST_CHECK(sample.T() == 200);
	BOOST_CHECK(sample.dz() == 110);
}

// Perform basic absorption calculations
BOOST_AUTO_TEST_CASE(absorber_calculations) {
	// First, note that absorbers must reference an atmoslayer class that provides 
	// p, T and dz.
	try {
		atmoslayer sample(100,200,110);
		BOOST_CHECK(sample.p() == 100);
		BOOST_CHECK(sample.T() == 200);
		BOOST_CHECK(sample.dz() == 110);
		// Construct an absorber
		abs_H2O ao;
		ao.setLayer(sample,0.1);

		double nu = absorber::_freqtowv(94.0);
		double ret = ao.deltaTau(nu);
		BOOST_CHECK(ret > 0); // Just make sure it's valid for now
		
		// Now try and place it in sample and run
		std::shared_ptr<absorber> cln(ao.clone());
		sample.absorbers.insert(cln);
		double retb = cln->deltaTau(nu);
		BOOST_CHECK(retb == ret);

		// Have sample actually do a tau calculation. With one element, verify consistency.
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
	}
}

// Verify absorption calculations



// Attempt to clone atmoslayer


// Create atmos and have it load a file
BOOST_AUTO_TEST_CASE(atmos_load_ryan) {
	using namespace std;
	using namespace atmos;
	try {
		string profilepath = "../profiles/trp.txt";
		rtmath::atmos::atmos trp;
		trp.loadProfileRyan(profilepath);
		double res = 0;
		res = trp.tau(94.0);
		cout << res << endl;
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
	}
}


BOOST_AUTO_TEST_SUITE_END();

