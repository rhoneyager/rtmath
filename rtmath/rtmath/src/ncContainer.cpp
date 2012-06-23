#include "../rtmath/Stdafx.h"
#include <memory>
//#include <netcdf.h>
#include <boost/filesystem.hpp>
#include <memory>
#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <unordered_map>
#include <boost/unordered_map.hpp> // Instead of std::unordered map, for now
#include <boost/bimap.hpp> // Support for bidirectional maps!!!
#include <limits.h>	
#include "../rtmath/error/error.h"
#include "../rtmath/netcdf/ncContainer.h"


#ifdef _WIN32
//#pragma comment(lib, "netcdf")
#endif

namespace rtmath {

	namespace netcdf
	{
		ncContainer::ncContainer(int ncid)
		{
			// Already open. Take ncid and set it.
			_init();
			_opened = true;
			_nFile = ncid;
		}

		ncContainer::ncContainer(const std::string &filename, int mode)
		{
			_init();
			_open(filename,0);
		}

		ncContainer::~ncContainer()
		{
			if (_opened) _close();
		}

		void ncContainer::_init()
		{
			_opened = false;
			_nFile = -1;
		}

		void ncContainer::_close()
		{
			throw rtmath::debug::xUnimplementedFunction();
			//nc_close(_nFile);
		}

		void ncContainer::_open(const std::string &filename, int mode)
		{
			throw rtmath::debug::xUnimplementedFunction();
			//int res;
			//res = nc_open(filename.c_str(), mode, &_nFile);
			//if (res) throw rtmath::debug::xUnknownFileFormat(filename.c_str());
		}
	}

}

