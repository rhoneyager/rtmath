#include "../rtmath/Stdafx.h"
#include <memory>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <sstream>
#include "../rtmath/absorb.h"
#include "../rtmath/atmoslayer.h"

namespace rtmath {
	
	namespace atmos {

		/*
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
		*/

		double absorber::_wvtofreq(double wvnum)
		{
			double f = wvnum * 2.99792458e8;
			// And, for appropriate dimensionality...
			f *= 1.e7;
			return f;
		}

		double absorber::_freqtowv(double f)
		{
			double wvnum = (f*1.e-7) / 2.99792458e8;
			return wvnum;
		}


		absorber::absorber(const atmoslayer &layer, double psfrac)
		{
			_init();
			setLayer(layer,psfrac);
		}

		void absorber::setLayer(const atmoslayer &layer, double psfrac)
		{
			_p = &layer._p;
			_T = &layer._T;
			_dz = &layer._dz;
			_psfrac = psfrac;
			_layer = &layer;
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
			_layer = 0;
		}



	}; // end namespace atmos

}; // end namespace rtmath

