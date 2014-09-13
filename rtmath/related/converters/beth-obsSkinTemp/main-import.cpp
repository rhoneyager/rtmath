/* This program ingests Beth's data and generates hdf5 files */
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <algorithm>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <Ryan_Debug/debug.h>

#include "observation.h"
#include "import.h"
#include "../../rtmath_silo_cpp/WritePoints.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace boost::filesystem;

	try {
		cerr << "obsSkinTemp-silo\n\n";

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);

		po::options_description desc("Allowed options");

		desc.add_options()
			("help,h", "produce help message")
			("input,i", po::value< string >(), "input hdf5 file")
			("output,o", po::value< string >(), "output silo file")
			;

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << "\n";
			if (message.size())
				cerr << message << "\n";
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");
		if (!vm.count("input")) doHelp("Need to specify input file(s).\n");
		string sinput = vm["input"].as< string >();
		string output;
		if (vm.count("output")) output = vm["output"].as< string >();
		else doHelp("Need to specify an output file.\n");

		std::vector<observation> obs;
		obs.reserve(12000000);
		observation minObs, maxObs;
		std::cerr << "Reading data" << std::endl;
		readHDF(sinput, obs, maxObs, minObs);

		if (!obs.size()) throw std::string("No data was successfully read!");

		std::cerr << "Constructing grid" << std::endl;
		// Construct a 3d grid for observations and number of observations
		Eigen::Array3i span, maxs, mins,
			scale = Eigen::Array3i::Ones();
		scale(0) = 1; scale(1) = 1; scale(2) = 1;
		span(0) = ((maxObs.temp - minObs.temp) / scale(0)) + 1;
		span(1) = ((maxObs.wbTemp - minObs.wbTemp) / scale(1)) + 1;
		span(2) = ((maxObs.skinTemp - minObs.skinTemp) / scale(2)) + 1;
		maxs(0) = maxObs.temp; maxs(1) = maxObs.wbTemp; maxs(2) = maxObs.skinTemp;
		mins(0) = minObs.temp; mins(1) = minObs.wbTemp; mins(2) = minObs.skinTemp;
		Eigen::VectorXf binTemps = Eigen::VectorXf::LinSpaced(span(0), mins(0), maxs(0));
		Eigen::VectorXf binWbTemps = Eigen::VectorXf::LinSpaced(span(1), mins(1), maxs(1));
		Eigen::VectorXf binSkinTemps = Eigen::VectorXf::LinSpaced(span(2), mins(2), maxs(2));

		std::cerr << "T\n" << binTemps << std::endl;
		std::cerr << "Wb\n" << binWbTemps << std::endl;
		std::cerr << "S\n" << binSkinTemps << std::endl;
		const char *dimLabels[] = { "Temperature", "Wet bulb temperature", "Skin temperature" };
		const char *dimUnits[] = { "C", "C", "C" };
		const int dimsizes[] = { (int)binTemps.size(), (int)binWbTemps.size(), (int)binSkinTemps.size() };
		const float *dims[] = { binTemps.data(), binWbTemps.data(), binSkinTemps.data() };

		//const int dimsizes[] = { (int)binSkinTemps.size(), (int)binWbTemps.size(), (int)binTemps.size() };
		//const float *dims[] = { binSkinTemps.data(), binWbTemps.data(), binTemps.data() };

		// Insert observations into the grid
		
		
		int nBins = span.prod();
		Eigen::MatrixXf prob(nBins, 1);
		prob.setZero();
		Eigen::MatrixXf numObs(nBins, 1);
		numObs.setZero();

		
		std::cerr << "Populating grids" << std::endl;

		auto getCoords = [&](int i)->Eigen::Array3i
		{
			Eigen::Array3i crd;
			int x, y, z;
			// Iterate first over z, then y, then x
			x = i / (span(0)*span(1));
			//crd(1) = (i % (span(2)*span(1))) / span(2);
			y = (i - (x*span(0)*span(1))) / span(0); // it's not (i - i), as x involves an INTEGER division!
			z = i % span(0);
			crd(2) = x; crd(1) = y; crd(0) = z;
			crd += mins;
			//x = crd(0); y = crd(1); z = crd(2);
			return crd;
		};
		auto getIndex = [&](Eigen::Array3i i) -> int
		{
			int res = 0;
			i -= mins;
			i = i / scale;
			res = span(0) * span(1) * i(2);
			res += span(0) * i(1);
			res += i(0);
			//int x = i(2), y = i(1), z = i(0);
			//res = z + (span(2) * (y + (span(1)*x)));
			return res;
		};

		for (size_t i = 0; i < obs.size(); ++i)
		{
			Eigen::Array3i a; a(0) = (int)obs[i].temp;
			a(1) = (int)obs[i].wbTemp; a(2) = (int)obs[i].skinTemp;
			int index = getIndex(a);
			prob(index, 0) += (float) obs[i].rain_snowFlag;
			numObs(index, 0)++;
		}
		for (int i = 0; i < nBins; ++i)
		{
			if (numObs(i, 0)) prob(i, 0) /= numObs(i, 0);
		}
		//prob.array() = prob.array() / numObs.array();

		// Open silo file
		std::cerr << "Writing output" << std::endl;
		std::shared_ptr<rtmath::plugins::silo::siloFile> sf = 
			rtmath::plugins::silo::siloFile::generate
			(output.c_str(), "Beth output file, gridded");

		// Create mesh
		auto mesh = sf->createRectilinearMesh<float>("Mesh", 3, dims, dimsizes, dimLabels, dimUnits);

		// Write fields
		mesh->writeData<float>("NumObs", numObs);
		mesh->writeData<float>("Snow_Probability", prob);
	}
	catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	catch (std::string &e)
	{
		cerr << e << endl;
		return 1;
	}
	return 0;
}

