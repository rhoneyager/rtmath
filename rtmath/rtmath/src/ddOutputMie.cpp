#include "../rtmath/Stdafx.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <complex>
#include <boost/filesystem.hpp>
#include "../rtmath/matrixop.h"
#include "../rtmath/error/error.h"
#include "../rtmath/ddscat/ddOutputSingle.h"
#include "../rtmath/ddscat/ddScattMatrix.h"
#include "../rtmath/units.h"
#include "../rtmath/quadrature.h"
#include "../rtmath/mie/ddOutputMie.h"
#include "../rtmath/mie/mieScattMatrix.h"

namespace mie {

	ddOutputMie::~ddOutputMie()
	{
	}

	void ddOutputMie::_init()
	{
	}

	ddOutputMie::ddOutputMie()
	{
		_init();
		lock();
	}

	ddOutputMie::ddOutputMie(double freq, double psize, double span)
	{
		_init();
		_freq = freq;
		_reff = psize;

		// TODO: less hardcode of theta
		for (double theta = 0; theta <= 180; theta += span)
		{
			std::shared_ptr<const rtmath::ddscat::ddScattMatrix> nm
				(new rtmath::ddscat::mieScattMatrix(freq,theta,psize));
			_insert(nm);
		}
		lock();
	}

} // end mie namespace
