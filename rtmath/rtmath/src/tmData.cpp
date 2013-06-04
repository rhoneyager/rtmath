#include "../rtmath/Stdafx.h"
#include <algorithm>
#include <cmath>
#include "../rtmath/ddscat/tmData.h"

namespace rtmath
{
	namespace tmatrix
	{
		tmStats::tmStats()
		{
		}

		tmData::tmData()
			:
				dipoleSpacing(0),
				T(0),
				nu(0),
				freq(0),
				sizep(0)
		{
			tstats = boost::shared_ptr<tmStats>(new tmStats);
		}


	}
}

