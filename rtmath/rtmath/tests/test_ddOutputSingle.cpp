#include <string>
#include <iostream>
#define BOOST_TEST_DYN_LINK
#include "globals.h"
#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <memory>

#include <Ryan_Serialization/serialization.h>
#include "../rtmath/ddscat/ddOutputSingle.h"

using namespace rtmath;
using namespace rtmath::ddscat;
using namespace std;
using namespace rtmath::config;
using namespace boost::filesystem;

BOOST_FIXTURE_TEST_SUITE(test_ddOutputSingle, ddOutputSingleLoader);

/// Test that we can load a standard .avg file
BOOST_AUTO_TEST_CASE(ddOutputSingle_loadAVG)
{
	try {
		oAvg1 = boost::shared_ptr<ddOutputSingle>
			(new ddOutputSingle(pAvg1.string()));
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		BOOST_FAIL("Exception thrown.");
	}
}

/// Test load of a compressed (zlib) avg file
BOOST_AUTO_TEST_CASE(ddOutputSingle_loadAVGCompressed)
{
	try {
		oAvg2 = boost::shared_ptr<ddOutputSingle>
			(new ddOutputSingle(pAvg2.string()));
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		BOOST_FAIL("Exception thrown.");
	}
}

/// Test load of a sca file
BOOST_AUTO_TEST_CASE(ddOutputSingle_loadSCA)
{
	try {
		oSca = boost::shared_ptr<ddOutputSingle>
			(new ddOutputSingle(pSca.string()));
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		BOOST_FAIL("Exception thrown.");
	}
}

/// Test load of fml file
BOOST_AUTO_TEST_CASE(ddOutputSingle_loadFML)
{
	try {
		oFml = boost::shared_ptr<ddOutputSingle>
			(new ddOutputSingle(pFml.string()));
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		BOOST_FAIL("Exception thrown.");
	}
}

/// Test load of xml file
BOOST_AUTO_TEST_CASE(ddOutputSingle_loadXML)
{
	try {
		oXml = boost::shared_ptr<ddOutputSingle>
			(new ddOutputSingle(pXml.string()));
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		BOOST_FAIL("Exception thrown.");
	}
}

/// Test load of xml.bz2 file, but with a custom file type specifier
BOOST_AUTO_TEST_CASE(ddOutputSingle_loadXMLtyped)
{
	try {
		oXmlbz2 = boost::shared_ptr<ddOutputSingle>
			(new ddOutputSingle(pXmlbz2.string(),".xml"));
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		BOOST_FAIL("Exception thrown.");
	}
}

/// Test serialization
BOOST_AUTO_TEST_CASE(ddOutputSingle_serialization)
{
	try {
		string sObj;
		ostringstream out;
		Ryan_Serialization::write<ddOutputSingle>(*oAvg1,out,"rtmath::ddscat::ddOutputSingle");
		Ryan_Serialization::write<ddOutputSingle>(*oAvg1,"ddOutputSingleTestSerialization.xml","rtmath::ddscat::ddOutputSingle");
		sObj = out.str();
		istringstream in(sObj);

		Ryan_Serialization::read<ddOutputSingle>(*oAvg2,in, "rtmath::ddscat::ddOutputSingle");
		
		BOOST_CHECK(oAvg1->aeff() == oAvg2->aeff());
		/// \todo Implement ddOutputSingle::operator==.
		//BOOST_CHECK(*oAvg1==*oAvg2);
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		BOOST_FAIL("Exception thrown.");
	}
}

/// Test serialization using builtin read/write functions
BOOST_AUTO_TEST_CASE(ddOutputSingle_serializationB)
{
	try {
		oAvg1->writeFile("ddOutputSingleTestSerializationB.xml");
		oSca->readFile("ddOutputSingleTestSerializationB.xml");
		BOOST_CHECK(oAvg1->aeff() == oSca->aeff());
		/// \todo Implement ddOutputSingle::operator==.
		//BOOST_CHECK(*oAvg1==*oSca);
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		BOOST_FAIL("Exception thrown.");
	}
}

BOOST_AUTO_TEST_SUITE_END();

