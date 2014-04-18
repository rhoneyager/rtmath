#include <boost/lexical_cast.hpp>
#include "observation.h"
#include "export.h"
#include "../../rtmath_hdf5_cpp/export-hdf5.h"

void exportToHDF(const std::string &filename, 
				 const std::vector<observation> &obs,
				 size_t rawSize, size_t chunkSize)
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
	addAttr<short, Group>(grpData, "Missing_Value", 9999);
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

	Eigen::MatrixXi mdataset((int) obs.size(), 9);
	for (int i=0; i<(int) obs.size();++i)
	{
		mdataset(i,0) = obs[i].lat;
		mdataset(i,1) = obs[i].lon;
		mdataset(i,2) = obs[i].sTime;
		mdataset(i,3) = obs[i].temp;
		mdataset(i,4) = obs[i].wbTemp;
		mdataset(i,5) = obs[i].rain_snowFlag;
		mdataset(i,6) = obs[i].pres;
		mdataset(i,7) = obs[i].skinTemp;
		mdataset(i,8) = obs[i].lapseRate;
	}
	std::shared_ptr<DSetCreatPropList> plist;
	plist = std::shared_ptr<DSetCreatPropList>(new DSetCreatPropList);
	int fillvalue = 9999;
	plist->setFillValue(PredType::NATIVE_INT, &fillvalue);
	if (chunkSize)
	{
		hsize_t chunking[2] = { (hsize_t) chunkSize, 9 };
		plist->setChunk(2, chunking);
		plist->setDeflate(6);
	}

	addDatasetEigen(grpData, "Observations", mdataset, plist);

	/*
	int row = 0;
	// Writing a compound datatype
	{
		hsize_t dim[1] = {obs.size()};
		DataSpace space(1, dim);
		CompType obsType(sizeof(observation));
		obsType.insertMember("Latitude", HOFFSET(observation, lat), PredType::NATIVE_SHORT);
		obsType.insertMember("Longitude", HOFFSET(observation, lon), PredType::NATIVE_USHORT);
		obsType.insertMember("Time", HOFFSET(observation, sTime), PredType::NATIVE_LONG);
		obsType.insertMember("Temperature", HOFFSET(observation, temp), PredType::NATIVE_SHORT);
		obsType.insertMember("Wet_Bulb_Temperature", HOFFSET(observation, wbTemp), PredType::NATIVE_SHORT);

		obsType.insertMember("Rain_SnowFlag", HOFFSET(observation, rain_snowFlag), PredType::NATIVE_SHORT);

		obsType.insertMember("Pressure", HOFFSET(observation, pres), PredType::NATIVE_SHORT);
		obsType.insertMember("Skin_Temperature", HOFFSET(observation, skinTemp), PredType::NATIVE_SHORT);
		obsType.insertMember("Lapse_Rate", HOFFSET(observation, lapseRate), PredType::NATIVE_SHORT);


		std::shared_ptr<DSetCreatPropList> plist;
		plist = std::shared_ptr<DSetCreatPropList>(new DSetCreatPropList);
		//int fillvalue = -1;
		//plist->setFillValue(PredType::NATIVE_INT, &fillvalue);
		if (chunkSize)
		{
			hsize_t chunking[2] = { (hsize_t) chunkSize, 1 };
			plist->setChunk(2, chunking);
			plist->setDeflate(6);
		}

		std::shared_ptr<DataSet> sdataset(new DataSet(grpData->createDataSet("Observations", obsType, space, *(plist.get()))));
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
		addAttr<size_t, DataSet>(sdataset, "Raw_size", rawSize); // Rows to read (on account of chunking)
	}
	*/

}


