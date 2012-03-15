/* db-catalog-ddscat
 * A program that recurses through a directory tree and catalogs
 * any ddscat.par files that it can find. Additionally, it notes
 * any .fml, .sca or .nc/.cdf files. It will read the ddscat.par
 * file. This is useful because it records in a database which runs
 * I have done, the shapes, frequencies, orientations (and density
 * of measurements) and where the necessary files are located.
 */

#include <iostream>
#include <cmath>
#include <fstream>
#include <set>
#include <map>
#include <string>
#include <sstream>
#include <vector>
#include <memory>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/asio/ip/host_name.hpp> // for system host name
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
//#include <netcdf.h>
#include <libpq-fe.h>

#include <stdlib.h>
#include "../../rtmath/rtmath/rtmath.h"

void doHelp();

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	try {
		cerr << "rtmath-db-catalog-ddscat" << endl;
		rtmath::debug::appEntry(argc, argv);
		config::parseParams p(argc,argv);
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		exit(1);
	}
	return 0;
}

void doHelp()
{
	exit(1);
}
