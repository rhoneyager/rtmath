#pragma once
#include "../defs.h"
#include <string>
#include <vector>
#include <map>
#include <set>
#include <complex>
//#include "../matrixop.h"
//#include "../interpolatable.h"
#include "../phaseFunc.h"
#include "ddScattMatrix.h"
//#include "../da/daStatic.h"

namespace rtmath
{
	namespace ddscat
	{
		/// \brief Class contains the output of a ddscat avg file. Now used for 
		/// comparisons and calculations to compare with tmatrix code.
		class DEPRECATED ddOutputAvg
		{
		public:
			ddOutputAvg(const std::string &filename);
			virtual ~ddOutputAvg();

			void loadFile(const std::string &filename);
		};
	}
}

