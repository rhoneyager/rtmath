#include "../rtmath/Stdafx.h"
#include <algorithm>
#include <cmath>
#include "../rtmath/ddscat/tmData.h"

namespace rtmath
{
	namespace tmatrix
	{
		tmIn::tmIn()
		{
			axi = 0;
			rat = 0;
			lam = 0;
			mrr = 0;
			mri = 0;
			eps = 0;
			ddelt = 0;
			alpha = 0;
			beta = 0;
			thet0 = 0;
			thet = 0;
			phi0 = 0;
			phi = 0;
			np = 0;
			ndgs = 0;
		}

		tmOut::tmOut()
		{
			std::fill_n(&P[0][0], 16, 0);
		}
	}
}

