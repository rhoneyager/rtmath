#pragma once

#include <memory>
#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <unordered_map>
#include <boost/unordered_map.hpp> // Instead of std::unordered map, for now
#include <boost/bimap.hpp> // Support for bidirectional maps!!!
#include "../enums.h"
#include "../matrixop.h"
#include <limits.h>	
#include "../defs.h"
#include "../error/error.h"
#include "../coords.h"

#include "ncDim.h"
#include "ncVar.h"

namespace rtmath {
	
	namespace netcdf
	{

		/* This container manages a netcdf file. It uses shared memory, and automatically closes the file when done.
		 * This container also keeps track of ncids for variables and dimensions. It is designed to be compatible 
		 * with netcdf version 3. No support for version 4 because it doesn't exist on Windows.
		 */
		class ncContainer : public std::enable_shared_from_this<ncContainer>
		{
		public:
			ncContainer(int ncid);
			ncContainer(const std::string &filename, int mode);
			virtual ~ncContainer();
			boost::bimap<int,std::string> dims, vars;
		private:
			void _init();
			void _open(const std::string &filename, int mode);
			void _close();
			bool _opened;
			int _nFile;
		};

	}

}
