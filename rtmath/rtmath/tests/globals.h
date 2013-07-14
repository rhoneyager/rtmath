#pragma once

/* This defines a global test fixture that locates the test directory paths. Many of the tests require IO for the 
 * test, but the location when testing is not the same as the install location.
 */
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include <memory>

namespace rtmath {
	namespace config {
		class configsegment;
	}
	namespace ddscat {
		class ddPar;
		class ddOutputSingle;
		class ddScattMatrix;
	};
}

struct globals
{
public:
	globals();
	~globals();
	static globals*& instance();
	void findDirs();
	void loadConfig();
	std::shared_ptr<rtmath::config::configsegment> cRoot;
	boost::filesystem::path pFullData, pTestData, pProfiles, pRtconf;
};


struct ddparloader
{
	ddparloader();
	~ddparloader();
	bool baseExists;
	bool defaultKey;
	boost::filesystem::path pBasePar;
	boost::filesystem::path pCWD;
	rtmath::ddscat::ddPar *testPar;
};

struct ddOutputSingleLoader
{
	ddOutputSingleLoader();
	~ddOutputSingleLoader();
	boost::filesystem::path pAvg1, pAvg2, pSca, pFml, pXml, pXmlbz2;
	boost::filesystem::path pCWD;
	boost::shared_ptr<rtmath::ddscat::ddOutputSingle> oAvg1, oAvg2, 
		oSca, oFml, oXml, oXmlbz2;
};
