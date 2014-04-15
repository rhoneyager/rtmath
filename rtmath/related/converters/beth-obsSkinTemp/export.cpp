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


	// Construct station table
	// Coop#, Lat, Lon, StnID, City, County, State, Start Date, End Date, numObs

	
	struct stationData
	{
		int COOP;
		float Lat, Lon, Elev;
		int numObs;
		// Using const char* because the compound writes have no string override.
		const char *StnID, *State, *City, *County;
		const char *start, *end;
		// Not written directly. These act as anchors for the const char* representation of the start and end times.
		string sstart, send;
	};

	vector<stationData> sdata(stations.size());

	int row = 0;
	for (const auto & s : stations)
	{
		auto &d = sdata[row];
		d.COOP = s.second.COOP;
		d.Lat = s.second.Lat;
		d.Lon = s.second.Lon;
		d.Elev = s.second.Elev;
		d.StnID = s.second.StnID.c_str();
		d.State = s.second.State.c_str();
		d.City = s.second.City.c_str();
		d.County = s.second.County.c_str();

		d.numObs = (int) s.second.numObs;
		d.sstart = boost::lexical_cast<string>(s.second.startTime);
		d.send = boost::lexical_cast<string>(s.second.endTime);
		d.start = d.sstart.c_str();
		d.end = d.send.c_str();

		row++;
	}
	// Writing a compound datatype
	{
		hsize_t dim[1] = {sdata.size()};
		DataSpace space(1, dim);
		CompType stationType(sizeof(stationData));
		stationType.insertMember("COOP", HOFFSET(stationData, COOP), PredType::NATIVE_INT);
		stationType.insertMember("Latitude", HOFFSET(stationData, Lat), PredType::NATIVE_FLOAT);
		stationType.insertMember("Longitude", HOFFSET(stationData, Lon), PredType::NATIVE_FLOAT);
		stationType.insertMember("Elevation", HOFFSET(stationData, Elev), PredType::NATIVE_FLOAT);

		//std::shared_ptr<H5::AtomType> strtype(new H5::StrType(0, H5T_VARIABLE));
		H5::StrType strtype(0, H5T_VARIABLE);

		stationType.insertMember("Station_ID", HOFFSET(stationData, StnID), strtype);
		stationType.insertMember("State", HOFFSET(stationData, State), strtype);
		stationType.insertMember("City", HOFFSET(stationData, City), strtype);
		stationType.insertMember("County", HOFFSET(stationData, County), strtype);
		stationType.insertMember("Start_Date", HOFFSET(stationData, start), strtype);
		stationType.insertMember("End_Date", HOFFSET(stationData, end), strtype);


		stationType.insertMember("Number_of_Observations", HOFFSET(stationData, numObs), PredType::NATIVE_INT);


		std::shared_ptr<DataSet> sdataset(new DataSet(grpData->createDataSet("Station_Data", stationType, space)));
		sdataset->write(sdata.data(), stationType);

		addAttr<string, DataSet>(sdataset, "COOP", "The COOP# of the site");
		addAttr<string, DataSet>(sdataset, "Latitude", "Site latitude (-90 to 90 degrees)");
		addAttr<string, DataSet>(sdataset, "Longitude", "Site longitude (-180 to 180 degrees)");
		addAttr<string, DataSet>(sdataset, "Elevation", "Site elevation");
		addAttr<string, DataSet>(sdataset, "Station_ID", "The station ID of the site");
		addAttr<string, DataSet>(sdataset, "State", "The state.");
		addAttr<string, DataSet>(sdataset, "City", "The city / station name.");
		addAttr<string, DataSet>(sdataset, "County", "The county.");
		addAttr<string, DataSet>(sdataset, "Start_Date", "The date (as a string) of the first reported observation");
		addAttr<string, DataSet>(sdataset, "End_Date", "The date (as a string) of the last reported observation.");
		addAttr<string, DataSet>(sdataset, "Number_of_Observations", "The number of observations over the measured interval (measurements reporting >= 0).");
	}


	// Construct the observation table
	Eigen::MatrixXf obs(stations.size(),daysB.size());
	obs.fill(-9999.f);

	row=0;
	for (const auto & s : stations)
	{
		for (int j=0; j< (int) s.second._obs.size(); ++j)
		{
			const auto &ob = s.second._obs.at(j);

			int index = (ob.first - start).days();
			obs(row,index) = ob.second;
		}
		row++;
	}

	hsize_t chunk_obs[2] = { (hsize_t) stations.size(), daysB.size() };
	auto plistObs = std::shared_ptr<DSetCreatPropList>(new DSetCreatPropList);
	plistObs->setChunk(2, chunk_obs);
	// If zlib is found
	plistObs->setDeflate(6);

	auto tblObs = addDatasetEigen<Eigen::MatrixXf, Group>(grpData, "Observations", obs, plistObs);
	addAttr<float, DataSet>(tblObs, "Missing_Value", -9999.0f);
	addAttr<string, DataSet>(tblObs, "Dimensions", "The rows correspond to the sites in the Station_Data table. The columns correspond to the dates in the Dates table.");
}


