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

// Test read of all values using the new interface
// (it calls the traditional interface, so that is also tested)
BOOST_AUTO_TEST_CASE(ddpar_read_new)
{
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
	// Needs support for multiple diel.tab files
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

	BOOST_CHECK( testPar->nAmbient() == 1.0);

	BOOST_CHECK ( testPar->PolState(0) == 0 );
	BOOST_CHECK ( testPar->PolState(1) == 0 );
	BOOST_CHECK ( testPar->PolState(2) == 1 );
	BOOST_CHECK ( testPar->PolState(3) == 0 );
	BOOST_CHECK ( testPar->PolState(4) == 0 );
	BOOST_CHECK ( testPar->PolState(5) == 0 );

	BOOST_CHECK( testPar->OrthPolState() == 2);
	BOOST_CHECK( testPar->writePol() == false);
	BOOST_CHECK( testPar->writeSca() == true);

	rotations rots;
	rotations rotscheck( 0, 0, 1, 0, 90, 10, 0, 0, 1 );
	testPar->getRots(rots);
	BOOST_CHECK( rots == rotscheck );

	BOOST_CHECK ( testPar->firstOri(0) == 0 );
	BOOST_CHECK ( testPar->firstOri(1) == 0 );
	BOOST_CHECK ( testPar->firstOri(2) == 0 );

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
	testPar->getCMDFRM(check);
	BOOST_CHECK( check == "LFRAME" );

	BOOST_CHECK ( testPar->numPlanes() == 2 );

	{
		double phi, thetan_min, thetan_max, dtheta;
		testPar->getPlane(1, phi, thetan_min, thetan_max, dtheta);
		BOOST_CHECK( phi == 0 );
		BOOST_CHECK( thetan_min == 0 );
		BOOST_CHECK( thetan_max == 180 );
		BOOST_CHECK( dtheta == 10 );
		testPar->getPlane(2, phi, thetan_min, thetan_max, dtheta);
		BOOST_CHECK( phi == 90 );
		BOOST_CHECK( thetan_min == 0 );
		BOOST_CHECK( thetan_max == 180 );
		BOOST_CHECK( dtheta == 10 );
	}
}

// Test write of all values using new interface
// No need to check quite everything, as the set functions are rather redundant
BOOST_AUTO_TEST_CASE(ddpar_write_new)
{
	std::string check;

	testPar->doTorques(true);
	BOOST_CHECK( testPar->doTorques() == true);
	
	testPar->setSolnMeth("PBCGST");
	testPar->getSolnMeth(check);
	BOOST_CHECK( check == "PBCGST" );

	testPar->setFFTsolver("GPFAFT");
	testPar->getFFTsolver(check);
	BOOST_CHECK( check == "GPFAFT" );

	testPar->setCalpha("LATTDR");
	testPar->getCalpha(check);
	BOOST_CHECK( check == "LATTDR");

	testPar->setBinning("ALLBIN");
	testPar->getBinning(check);
	BOOST_CHECK( check == "ALLBIN");

	testPar->Imem(0, 102);
	testPar->Imem(1, 103);
	testPar->Imem(2, 104);
	BOOST_CHECK( testPar->Imem(0) == 102);
	BOOST_CHECK( testPar->Imem(1) == 103);
	BOOST_CHECK( testPar->Imem(2) == 104);

	testPar->setShape("ANIFRMFIL");
	testPar->getShape(check);
	BOOST_CHECK( check == "ANIFRMFIL" );

	testPar->shpar(0, 99);
	testPar->shpar(1, 98);
	testPar->shpar(2, 97);
	BOOST_CHECK( testPar->shpar(0) == 99);
	BOOST_CHECK( testPar->shpar(1) == 98);
	BOOST_CHECK( testPar->shpar(2) == 97);

	// diel.tab stuff
	// Needs support for multiple diel.tab files
	//{
	//}

	testPar->doNearField(true);
	BOOST_CHECK( testPar->doNearField() == true );

	testPar->near(0, 0.1);
	testPar->near(1, 0.2);
	testPar->near(2, 0.3);
	testPar->near(3, 0.4);
	testPar->near(4, 0.5);
	testPar->near(5, 0.6);
	BOOST_CHECK( testPar->near(0) == 0.1);
	BOOST_CHECK( testPar->near(1) == 0.2);
	BOOST_CHECK( testPar->near(2) == 0.3);
	BOOST_CHECK( testPar->near(3) == 0.4);
	BOOST_CHECK( testPar->near(4) == 0.5);
	BOOST_CHECK( testPar->near(5) == 0.6);

	testPar->maxTol(2.e-5);
	BOOST_CHECK( testPar->maxTol() == 2.e-5);
	testPar->maxIter(301);
	BOOST_CHECK( testPar->maxIter() == 301);
	testPar->gamma(7.e-3);
	BOOST_CHECK( testPar->gamma() == 7.e-3);
	testPar->etasca(0.425);
	BOOST_CHECK( testPar->etasca() == 0.425);

	{
		testPar->setWavelengths(3100,3200,3,"INV");
		double min, max;
		size_t n;
		testPar->getWavelengths(min,max,n,check);
		BOOST_CHECK( min == 3100 );
		BOOST_CHECK( max == 3200 );
		BOOST_CHECK( n == 3 );
		BOOST_CHECK( check == "INV" );

		testPar->setAeff(600,700,2,"LOG");
		testPar->getAeff(min,max,n,check);
		BOOST_CHECK( min == 600 );
		BOOST_CHECK( max == 700 );
		BOOST_CHECK( n == 2 );
		BOOST_CHECK( check == "LOG" );
	}

	testPar->nAmbient(1.33);
	BOOST_CHECK( testPar->nAmbient() == 1.33);

	testPar->PolState(0, 1);
	testPar->PolState(1, 0);
	testPar->PolState(2, 0);
	testPar->PolState(3, 1);
	testPar->PolState(4, 1);
	testPar->PolState(5, 0);
	BOOST_CHECK ( testPar->PolState(0) == 1 );
	BOOST_CHECK ( testPar->PolState(1) == 0 );
	BOOST_CHECK ( testPar->PolState(2) == 0 );
	BOOST_CHECK ( testPar->PolState(3) == 1 );
	BOOST_CHECK ( testPar->PolState(4) == 1 );
	BOOST_CHECK ( testPar->PolState(5) == 0 );

	testPar->OrthPolState(1);
	BOOST_CHECK( testPar->OrthPolState() == 1);
	testPar->writePol(true);
	BOOST_CHECK( testPar->writePol() == true);
	testPar->writeSca(true);
	BOOST_CHECK( testPar->writeSca() == true);

	rotations rots( 1, 1, 1, 30, 89, 11, 2, 3, 4 ), rotscheck;
	testPar->setRots(rots);
	testPar->getRots(rotscheck);
	BOOST_CHECK( rots == rotscheck );

	testPar->firstOri(0, 1);
	testPar->firstOri(1, 1);
	testPar->firstOri(2, 2);
	BOOST_CHECK ( testPar->firstOri(0) == 1 );
	BOOST_CHECK ( testPar->firstOri(1) == 1 );
	BOOST_CHECK ( testPar->firstOri(2) == 2 );

	testPar->setCMDFRM("TFRAME");
	testPar->getCMDFRM(check);
	BOOST_CHECK( check == "TFRAME" );

	BOOST_CHECK ( testPar->numPlanes() == 2 );

	{
		testPar->setPlane(2, 3, 4, 5, 6);
		double phi, thetan_min, thetan_max, dtheta;
		testPar->getPlane(2, phi, thetan_min, thetan_max, dtheta);
		BOOST_CHECK( phi == 3 );
		BOOST_CHECK( thetan_min == 4 );
		BOOST_CHECK( thetan_max == 5 );
		BOOST_CHECK( dtheta == 6 );
	}
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

