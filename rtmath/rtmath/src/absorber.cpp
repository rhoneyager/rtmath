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

		absorber::absorber(int molnum)
		{
			_init();
			_molnum = molnum;
		}

		absorber::absorber(const std::string &molecule)
		{
			_init();
			_molecule = molecule;
		}

		absorber::~absorber()
		{
		}

		void absorber::_init()
		{
			_p = 0;
			_T = 0;
			_dz = 0;
			_psfrac = 0;
			_molnum = 0;
			_molecule = "";
		}



	}; // end namespace atmos

}; // end namespace rtmath

