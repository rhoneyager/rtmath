#include "import.h"

#include <boost/lexical_cast.hpp>
#include "observation.h"
#include "../../rtmath_hdf5_cpp/export-hdf5.h"

void readHDF(const std::string &filename,
	std::vector<observation> &obs,
	observation& maxObs, observation& minObs)
{
	using namespace H5;
	Exception::dontPrint();
	using namespace rtmath::plugins::hdf5;
	using namespace std;
	std::shared_ptr<H5File> file(new H5File(filename, H5F_ACC_RDONLY));
	shared_ptr<Group> grpData = openGroup(file, "landObsSkintempLR");

	Eigen::Matrix<int, Eigen::Dynamic, 9> mdataset;
	readDatasetEigen<Eigen::Matrix<int, Eigen::Dynamic, 9>, Group>
		(grpData, "Observations", mdataset);

	for (int i = 0; i<(int)mdataset.rows(); ++i)
	{
		observation lobs;
		lobs.lat = mdataset(i, 0);
		lobs.lon = mdataset(i, 1);
		lobs.sTime = mdataset(i, 2);
		lobs.temp = mdataset(i, 3) / 1000;
		lobs.wbTemp = mdataset(i, 4) / 1000;
		lobs.rain_snowFlag = mdataset(i, 5);
		lobs.pres = mdataset(i, 6) / 10;
		lobs.skinTemp = (int) (((float) mdataset(i, 7) / (float) 10000) - 273.15f);
		lobs.lapseRate = mdataset(i, 8) / 100;

		if (lobs.lat == 0) continue;

		// Max/min bounds
#define doit(x) if(minObs.x > lobs.x) minObs.x = lobs.x; if (maxObs.x < lobs.x) maxObs.x = lobs.x;
		doit(lat);
		doit(lon);
		doit(sTime);
		doit(temp);
		doit(wbTemp);
		doit(rain_snowFlag);
		doit(pres);
		doit(skinTemp);
		doit(lapseRate);
#undef doit

		obs.push_back(std::move(lobs));
	}
	

}


