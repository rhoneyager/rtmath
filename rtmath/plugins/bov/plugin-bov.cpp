/// \brief Provides bov file reading and writing to the shapefile class
#define _SCL_SECURE_NO_WARNINGS

#include <cstdio>
#include <cstring>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"

void dllEntry();
rtmath_plugin_init(dllEntry);

void dllEntry()
{
	static const rtmath::registry::DLLpreamble id(
		"Plugin-BOV",
		"Example plugin to provide shapefile class with the ability to "
		"read and write bov files.",
		"477DF60F-EEAA-45D1-9E4F-272630E901F9");
	rtmath_registry_register_dll(id);
	std::cerr << "plugin-bov dll loaded!\n\n";
}

void write_bov_shapefile(const rtmath::ddscat::shapefile &shp, const std::string &prefix)
{
	using namespace boost::filesystem;
	using std::string;
	using std::ofstream;

	path pPrefix(prefix);
	if (pPrefix.has_extension())
	{
		if (pPrefix.extension() == ".bov") pPrefix.replace_extension();
	}

	string sDataFile = pPrefix.string(); sDataFile.append(".dat");
	string sCtrlFile = pPrefix.string(); sCtrlFile.append(".bov");
	path pDatafile = path(sDataFile).filename();


	size_t maxX = static_cast<size_t>(shp.maxs(0)), maxY = static_cast<size_t>(shp.maxs(1)), maxZ = static_cast<size_t>(shp.maxs(2));
	size_t minX = static_cast<size_t>(shp.mins(0)), minY = static_cast<size_t>(shp.mins(1)), minZ = static_cast<size_t>(shp.mins(2));
	size_t spanX = maxX-minX+1, spanY = maxY-minY+1, spanZ = maxZ-minZ+1;

	// First, write the control file
	ofstream oct(sCtrlFile.c_str(), std::ios::binary | std::ios::out);
	oct << "TIME: 0\n"
		"DATA_FILE: " << pDatafile.string() << "\n"
		"# The data file size corresponds to the raw flake dimensions\n"
		"DATA_SIZE: " << spanX << " " << spanY << " " << spanZ << "\n"
		"# Allowable values for DATA_FORMAT are: BYTE,SHORT,INT,FLOAT,DOUBLE\n"
		"DATA_FORMAT: SHORT\n"
		"VARIABLE: Composition\n"
		"# Endian representation of the computer that created the data.\n"
		"# Intel is LITTLE, many other processors are BIG.\n"
		"DATA_ENDIAN: LITTLE\n"
		"# Centering refers to how the data is distributed in a cell. If you\n"
		"# give \"zonal\" then it’s 1 data value per zone. Otherwise the data\n"
		"# will be centered at the nodes.\n"
		"CENTERING: zonal\n"
		"# BRICK_ORIGIN lets you specify a new coordinate system origin for\n"
		"# the mesh that will be created to suit your data.\n"
		"BRICK_ORIGIN: "
		<< static_cast<int>(shp.x0(0)) << " "
		<< static_cast<int>(shp.x0(1)) << " "
		<< static_cast<int>(shp.x0(2)) << "\n"
		"# BRICK_SIZE lets you specify the size of the brick.\n"
		"BRICK_SIZE: 10. 10. 10.\n"
		"# DATA_COMPONENTS: is optional and tells the BOV reader how many\n"
		"# components your data has. 1=scalar, 2=complex number, 3=vector,\n"
		"# 4 and beyond indicate an array variable. You can use \"COMPLEX\"\n"
		"# instead of \"2\" for complex numbers. When your data consists of\n"
		"# multiple components, all components for a cell or node are written\n"
		"# sequentially to the file before going to the next cell or node.\n"
		"DATA_COMPONENTS: 1\n";
	// Then, write the data file
	//ofstream out(sDataFile.c_str(), std::ios::binary | std::ios::out);
	FILE * pOut;
	pOut = fopen(sDataFile.c_str(), "wb");
	// Allocate an array of the correct size
	const size_t size = spanX * spanY * spanZ;
	std::unique_ptr<short[]> array(new short[size]);
	std::fill_n(array.get(), size*1, 0);

	auto getIndex = [&](float x, float y, float z) -> size_t
	{
		size_t index = 0;
		size_t sX = (size_t) (x-minX);
		size_t sY = (size_t) (y-minY);
		size_t sZ = (size_t) (z-minZ);
		//index = (sX * (spanY * spanZ)) + (sY * spanZ) + sZ;
		index = (sZ * (spanX * spanY)) + (sY * spanX) + sX;
		return index;
	};

	for (size_t i=0; i < shp.numPoints; ++i)
	{
		auto crdsm = shp.latticePts.block<1,3>(i,0);
		float x = crdsm(0), y = crdsm(1), z = crdsm(2);
		auto crdsi = shp.latticePtsRi.block<1,3>(i,0);
		short diel = static_cast<short>(crdsi(0));

		size_t start = getIndex(x,y,z);

		array[start+0] = diel;
		//array[start+1] = static_cast<short>(crdsi(1));
		//array[start+2] = static_cast<short>(crdsi(2));
	}
	fwrite((void*)array.get(), sizeof(short), size, pOut);
	fclose(pOut);
}

void read_bov_shapefile()
{
}

/*
int main(int argc, char** argv)
{
using namespace std;
using namespace rtmath;
using namespace boost::filesystem;

try {
cerr << "rtmath-shape-extract\n\n";

namespace po = boost::program_options;

po::positional_options_description p;
p.add("input", -1);

po::options_description desc("Allowed options"), cmdline("Command-line options"), 
config("Config options"), hidden("Hidden options"), oall("all options");

cmdline.add_options()
("help,h", "produce help message")
("input,i", po::value< string >(), "input shape file")
("bov,b", po::value<string>(), "output bov file prefix")
;

desc.add(cmdline).add(config);
oall.add(cmdline).add(config).add(hidden);

po::variables_map vm;
po::store(po::command_line_parser(argc, argv).
options(oall).positional(p).run(), vm);
po::notify(vm);

auto doHelp = [&](const std::string &message)
{
cerr << desc << "\n";
if (message.size()) cerr << message << endl;
exit(1);
};

if (vm.count("help") || argc == 1) doHelp("");
if (!vm.count("input")) doHelp("Need to specify an input file.");
string input = vm["input"].as<string>();
cerr << "Reading input shape file " << input << endl;
rtmath::ddscat::shapefile shp;
shp.read(input);

if (vm.count("bov"))
{
string bPrefix = vm["bov"].as<string>();
cerr << "Writing BOV files with prefix " << bPrefix << endl;
//shp.write(string(bPrefix).append("-orig.dat"));
shp.writeBOV(bPrefix);
}

}
catch (rtmath::debug::xError &err)
{
err.Display();
cerr << endl;
return 1;
} catch (std::exception &e)
{
cerr << e.what() << endl;
return 1;
}
return 0;
}
*/
