#include <iostream>
#include "../rtmath/rtmath.h"
#include "../rtmath/rtmath-base.h"
#include "../rtmath/mie.h"
#include "../rtmath/rayleigh.h"
#include <complex>
#include <time.h>
//#include <netcdf.h>
//#include <netcdfcpp.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <fstream>
#include <stdio.h>
#include <omp.h>
#include <memory>
#include <string>

#include "../rtmath/debug_mem.h"
#include "../rtmath/damatrix_quad.h"
#include "../rtmath/rayleigh.h"
#include "../rtmath/ddscat2.h"


int main(int argc, char* argv[])
{
	using namespace std;
	using namespace rtmath;
	using namespace rtmath::ddscat;

	// Take ddscat name or path from argv, and attempt to load the files
	if (argc == 1) return 1;
	string file(argv[1]);
	cout << "Loading from " << file << endl << endl;
	//ddOutputSingle a(file);
	ddOutput a(file);
	std::map<ddCoords3, ddOutputSingle, ddCoordsComp>::const_iterator it;
	cout << "Loaded " << a._data.size() << " files" << endl;
	return 0;
}
