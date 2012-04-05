#include "../rtmath/Stdafx.h"
#include <memory>
#include <math.h>
#include "../rtmath/gridded/gridded.h"
#include "../rtmath/gridded/gridCDF.h"

namespace rtmath {

	namespace griddata
	{
		gridCDF::gridCDF()
		{
		}

		gridCDF::~gridCDF()
		{
		}

		std::shared_ptr<const matrixop> gridCDF::eval(const coords::cyclic<double> &start, 
			const coords::cyclic<double> &span) const
		{
			return nullptr;
		}
	}
}

