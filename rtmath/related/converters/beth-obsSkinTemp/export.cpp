#include <boost/lexical_cast.hpp>
#include "observation.h"
#include "export.h"
#include "../../rtmath_hdf5_cpp/export-hdf5.h"

void exportToHDF(const std::string &filename, 
				 const std::vector<observation> &obs)
{
	using namespace H5;
	Exception::dontPrint();
	using namespace rtmath::plugins::hdf5;
	using namespace std;
	std::shared_ptr<H5File> file(new H5File(filename, H5F_ACC_TRUNC ));
	shared_ptr<Group> grpData = openOrCreateGroup(file, "landObsSkintempLR");

	// Set some basic global attributes
	addAttr<string, Group>(grpData, "Source", "Imported data "
		"from Beth files");

	/// \todo Add ingest filenames!

	/// \todo Add date range + domain information
	//string sstart = boost::lexical_cast<string>(start);
	//string send = boost::lexical_cast<string>(end);

	//addAttr<string, Group>(grpData, "Start_Date", sstart);
	//addAttr<string, Group>(grpData, "End_Date", send);
	addAttr<float, Group>(grpData, "Missing_Value", -99.9f);
	/*
	// Construct time tables (as date (string))
	//auto numDays = (end - start).days() + 1;
	
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

	*/

	int row = 0;
	// Writing a compound datatype
	{
		hsize_t dim[1] = {obs.size()};
		DataSpace space(1, dim);
		CompType obsType(sizeof(observation));
		obsType.insertMember("Latitude", HOFFSET(observation, lat), PredType::NATIVE_FLOAT);
		obsType.insertMember("Longitude", HOFFSET(observation, lon), PredType::NATIVE_FLOAT);
		obsType.insertMember("Time", HOFFSET(observation, sTime), PredType::NATIVE_LONG);
		obsType.insertMember("Temperature", HOFFSET(observation, temp), PredType::NATIVE_FLOAT);
		obsType.insertMember("Wet_Bulb_Temperature", HOFFSET(observation, wbTemp), PredType::NATIVE_FLOAT);

		obsType.insertMember("Rain_SnowFlag", HOFFSET(observation, rain_snowFlag), PredType::NATIVE_INT);

		obsType.insertMember("Pressure", HOFFSET(observation, pres), PredType::NATIVE_FLOAT);
		obsType.insertMember("Skin_Temperature", HOFFSET(observation, skinTemp), PredType::NATIVE_FLOAT);
		obsType.insertMember("Lapse_Rate", HOFFSET(observation, lapseRate), PredType::NATIVE_FLOAT);


		std::shared_ptr<DataSet> sdataset(new DataSet(grpData->createDataSet("Observations", obsType, space)));
		sdataset->write(obs.data(), obsType);

		addAttr<string, DataSet>(sdataset, "Latitude", "Site latitude (-90 to 90 degrees)");
		addAttr<string, DataSet>(sdataset, "Longitude", "Site longitude (-180 to 180 degrees)");
		addAttr<string, DataSet>(sdataset, "Time", "Observation Time (seconds since 1/1/1970 00:00 GMT)");
		addAttr<string, DataSet>(sdataset, "Temperature", "Temperature in Celsius");
		addAttr<string, DataSet>(sdataset, "Wet_Bulb_Temperature", "Wet bulb temperature in Celsius");
		addAttr<string, DataSet>(sdataset, "Rain_SnowFlag", "flag = 0 indicates rain; flag = 1 indicates snow");
		addAttr<string, DataSet>(sdataset, "Pressure", "Pressure in hPa");
		addAttr<string, DataSet>(sdataset, "Skin_Temperature", "Skin temperature (K)");
		addAttr<string, DataSet>(sdataset, "Lapse_Rate", "Lapse rate (UNKNOWN units, missing value is 99.99f.)");
	}


	/*
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
	*/
}


