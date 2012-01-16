#include "../rtmath/Stdafx.h"
#include <memory>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <sstream>
#include "../rtmath/atmos.h"

namespace rtmath {
	
	namespace atmos {

		void atmoslayer::_init()
		{
			_p = 0;
			_T = 0;
			_dz = 0;
		}



	}; // end namespace atmos

}; // end namespace rtmath

