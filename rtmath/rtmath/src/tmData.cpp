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
			axi = 10;
			rat = 0.1;
			lam = 360;
			mrr = 1.5;
			mri = 0.02;
			eps = 0.5;
			ddelt = 0.001;
			alpha = 145;
			beta = 52;
			thet0 = 56;
			thet = 65;
			phi0 = 114;
			phi = 128;
			np = -1;
			ndgs = 4;
		}

		tmOut::tmOut()
		{
			std::fill_n(&P[0][0], 16, 0);
		}
	}
}

