#include <string>
#include <iostream>
#define BOOST_TEST_DYN_LINK
#include "globals.h"
#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <memory>

#include "../rtmath/rtmath.h"

using namespace rtmath;
using namespace rtmath::ddscat;
using namespace std;
using namespace rtmath::config;
using namespace boost::filesystem;

/* Notes:
 * no facility for testing / comparing versioned reads and writes
 */

struct ddparloader
{
	ddparloader()
		: defaultKey(false),
		baseExists(false),
		testPar(nullptr)
	{
		
		// Check that there is a default key
		string sBasePar;
		defaultKey = globals::instance()->cRoot->getVal<string>("ddscat/DefaultFile", sBasePar);
		if (!defaultKey) BOOST_FAIL("ddscat/DefaultFile key missing from testing rtmath.conf");

		// Check that default key points to a file
		string scwd;
		globals::instance()->cRoot->getCWD(scwd);
		pCWD = path(scwd).remove_filename();

		pBasePar = boost::filesystem::absolute(path(sBasePar), pCWD);
		if (!exists(pBasePar)) BOOST_FAIL("ddscat/DefaultFile key refers to nonexistant file");
		if (is_directory(pBasePar)) BOOST_FAIL("ddscat/DefaultFile key points to a directory, not a file");

		// Attempt to load the default
		try {
			testPar = rtmath::ddscat::ddPar::defaultInstance();
			//testPar->_populateDefaults();
		} catch (std::exception &e)
		{
			cerr << e.what() << endl;
			BOOST_FAIL("Unable to load default testing ddscat.par file. Cannot do ddPar tests.");
		}
		
	}
	~ddparloader()
	{
	}
	bool baseExists;
	bool defaultKey;
	path pBasePar;
	path pCWD;
	rtmath::ddscat::ddPar *testPar;
};


//BOOST_AUTO_TEST_SUITE(test_ddpar);
BOOST_FIXTURE_TEST_SUITE(test_ddpar, ddparloader);
// globals::instance()->

// Test that ddscat.par file can load
BOOST_AUTO_TEST_CASE(ddpar_defaultLoad)
{
	try {
		rtmath::ddscat::ddPar par(pBasePar.string());
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		BOOST_FAIL("Exception thrown.");
	}
}

// Test read of all values using the traditional interface
BOOST_AUTO_TEST_CASE(ddpar_read_traditional)
{
	BOOST_TEST_MESSAGE("   ddpar_read_traditional unimplemented");
}

// Test read of all values using  the new interface
BOOST_AUTO_TEST_CASE(ddpar_read_new)
{
	BOOST_TEST_MESSAGE("   ddpar_read_new is incomplete");

	std::string check;
	BOOST_CHECK( testPar->doTorques() == false);
	
	testPar->getSolnMeth(check);
	BOOST_CHECK( check == "PBCGS2" );

	testPar->getFFTsolver(check);
	BOOST_CHECK( check == "GPFAFT" );

	testPar->getCalpha(check);
	BOOST_CHECK( check == "GKDLDR");

	testPar->getBinning(check);
	BOOST_CHECK( check == "NOTBIN");

	BOOST_CHECK( testPar->Imem(0) == 101);
	BOOST_CHECK( testPar->Imem(1) == 101);
	BOOST_CHECK( testPar->Imem(2) == 101);

	testPar->getShape(check);
	BOOST_CHECK( check == "FROM_FILE" );

	BOOST_CHECK( testPar->shpar(0) == 101);
	BOOST_CHECK( testPar->shpar(1) == 101);
	BOOST_CHECK( testPar->shpar(2) == 101);

	// diel.tab stuff
	//{
	//}

	BOOST_CHECK( testPar->doNearField() == false );
	BOOST_CHECK( testPar->near(0) == 0);
	BOOST_CHECK( testPar->near(1) == 0);
	BOOST_CHECK( testPar->near(2) == 0);
	BOOST_CHECK( testPar->near(3) == 0);
	BOOST_CHECK( testPar->near(4) == 0);
	BOOST_CHECK( testPar->near(5) == 0);

	BOOST_CHECK( testPar->maxTol() == 1.e-5);
	BOOST_CHECK( testPar->maxIter() == 300);
	BOOST_CHECK( testPar->gamma() == 5.e-3);
	BOOST_CHECK( testPar->etasca() == 0.5);

	{
		double min, max;
		size_t n;
		testPar->getWavelengths(min,max,n,check);
		BOOST_CHECK( min == 3189.28 );
		BOOST_CHECK( max == 3189.28 );
		BOOST_CHECK( n == 1 );
		BOOST_CHECK( check == "LIN" );

		testPar->getAeff(min,max,n,check);
		BOOST_CHECK( min == 616.22064 );
		BOOST_CHECK( max == 616.22064 );
		BOOST_CHECK( n == 1 );
		BOOST_CHECK( check == "LIN" );
	}

	BOOST_CHECK( testPar->nambient() == 1.0);
	BOOST_CHECK( testPar->OrthPolState() == 2);
	BOOST_CHECK( testPar->writeSca() == true);

	rotations rots;
	rotations rotscheck( 0, 0, 1, 0, 90, 10, 0, 0, 1 );
	testPar->getRots(rots);
	BOOST_CHECK( rots == rotscheck );

	// IRAD

	{
		std::set<size_t> SIJ;
		testPar->getSIJ(SIJ);
		BOOST_CHECK( SIJ.size() == 6 );
		BOOST_CHECK( SIJ.count(11) );
		BOOST_CHECK( SIJ.count(12) );
		BOOST_CHECK( SIJ.count(21) );
		BOOST_CHECK( SIJ.count(22) );
		BOOST_CHECK( SIJ.count(31) );
		BOOST_CHECK( SIJ.count(41) );
	}

	// Scattering planes

}

// Test write of values using traditional interface
BOOST_AUTO_TEST_CASE(ddpar_write_traditional)
{
	BOOST_TEST_MESSAGE("   ddpar_write_traditional unimplemented");
}

// Test write of all values using new interface
BOOST_AUTO_TEST_CASE(ddpar_write_new)
{
	BOOST_TEST_MESSAGE("   ddpar_write_new unimplemented");
}

// Test scattering plane read
BOOST_AUTO_TEST_CASE(ddpar_scaPlaneRead)
{
	BOOST_TEST_MESSAGE("   ddpar_scaPlaneRead unimplemented");
}

// Test scattering plane write
BOOST_AUTO_TEST_CASE(ddpar_scaPlaneWrite)
{
	BOOST_TEST_MESSAGE("   ddpar_scaPlaneWrite unimplemented");
}

// Test ddpar write and read
BOOST_AUTO_TEST_CASE(ddpar_io)
{
	ddPar a(*testPar);
	ddPar b;

	string sObj;
	ostringstream out;
	a.write(out);
	sObj = out.str();
	istringstream in(sObj);
	b.read(in);

//a.writeFile("ddscat.a.par");
//b.writeFile("ddscat.b.par");

	BOOST_CHECK(a==b);
}


// Test ddpar serialization
BOOST_AUTO_TEST_CASE(ddpar_serialization)
{
	ddPar a(*testPar);
	ddPar b;

	string sObj;
	ostringstream out;
	rtmath::serialization::write<ddPar>(a,out);
	sObj = out.str();
	istringstream in(sObj);

//rtmath::serialization::write<ddPar>(a,"a.ddscat.par");

	rtmath::serialization::read<ddPar>(b,in);
//rtmath::serialization::write<ddPar>(b,"b.ddscat.par");

	BOOST_CHECK(a==b);
}


BOOST_AUTO_TEST_SUITE_END();

