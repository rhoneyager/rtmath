#pragma once

#include <memory>
#include <map>
#include <unordered_map>
#include <boost/unordered_map.hpp> // Instead of std::unordered map, for now
#include "../enums.h"
#include "../matrixop.h"
#include <limits.h>	
#include "../defs.h"
#include "../error/error.h"
#include "../coords.h"

namespace rtmath {
	
	namespace griddata
	{

		/* The gridCDF class contains a single netcdf variable's gridded data. The eval function 
		 * allows for a selection of the gridded space to be retreived. This is not cached here, 
		 * as there is no need, and it saves memory. The subsequent gridded class calls also do not 
		 * cache by default.
		 */
		class gridCDF : public gridded
		{
		public:
			gridCDF();
			virtual ~gridCDF();
			virtual std::shared_ptr<const matrixop> eval(const coords::cyclic<double> &start, 
				const coords::cyclic<double> &span) const;
		};

	}

}

