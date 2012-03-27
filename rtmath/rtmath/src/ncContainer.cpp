#include "../rtmath/Stdafx.h"
#include <memory>
#include <netcdf.h>
#include <boost/filesystem.hpp>
#include "../rtmath/error/error.h"
#include "../rtmath/netcdf/ncContainer.h"


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
			nc_close(_nFile);
		}

		void ncContainer::_open(const std::string &filename, int mode)
		{
			int res;
			res = nc_open(filename.c_str(), mode, &_nFile);
			if (res) throw rtmath::debug::xUnknownFileFormat(filename.c_str());
		}
	}

}

