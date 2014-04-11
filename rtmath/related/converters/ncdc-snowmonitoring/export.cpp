#include <boost/lexical_cast.hpp>
#include "station.h"
#include "export.h"
#include "export-hdf5.h"

void exportToHDF(const std::string &filename, 
				 const boost::gregorian::date &start,
				 const boost::gregorian::date &end,
				 const std::map<int,station> &stations)
{
	using namespace H5;
	Exception::dontPrint();
	using namespace exports::hdf5;
	using namespace std;
	std::shared_ptr<H5File> file(new H5File(filename, H5F_ACC_TRUNC ));
	shared_ptr<Group> grpData = openOrCreateGroup(file, "NCDCsnowData");

	// Set some basic global attributes
	addAttr<string, Group>(grpData, "Source", "Imported data "
		"from http://www.ncdc.noaa.gov/snow-and-ice/dly-data.php");
	string sstart = boost::lexical_cast<string>(start);
	string send = boost::lexical_cast<string>(end);

	addAttr<string, Group>(grpData, "Start_Date", sstart);
	addAttr<string, Group>(grpData, "End_Date", send);
	addAttr<int, Group>(grpData, "Missing_Value", -9999);

	// Construct time tables (as date (string))
	auto numDays = (end - start).days() + 1;
	//Eigen::MatrixXi daysA = Eigen::MatrixXi::LinSpaced(0, (int) numDays);

	vector<string> daystrs(numDays);
	vector<const char*> daysB(numDays);
	for (size_t i=0; i<daystrs.size(); ++i)
	{
		daystrs[i] = boost::lexical_cast<string>(start+boost::gregorian::days((long) i));
		daysB[i] = daystrs[i].c_str();
	}

	auto tblDates = addDatasetArray<const char*, Group>(grpData, "Dates", daysB.size(), 1, daysB.data());
	addAttr<string, DataSet>(tblDates, "Start_Date", sstart);
	addAttr<string, DataSet>(tblDates, "End_Date", send);


	// Construct station table (compound data)
	// Coop#, Lat, Lon, StnID, City, County, State, Start Date, End Date, numObs

	// Construct the observation table


}


