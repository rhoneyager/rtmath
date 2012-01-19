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

		double absorber::_wvtofreq(double nu)
		{
			double f = nu * 2.99792458e10/1.e9; // COnvert cm^-1 to GHz
			return f;
		}

		double absorber::_freqtowv(double f)
		{
			double nu = f*1.e9/2.997925e10; // convert GHz to cm^-1
			return nu;
		}

		double absorber::_Vden(double T, double RH)
		{
			const double RV = 461.5;
			double es = 6.1365*exp(17.502*(T-273.15)/(T-32.18));
			double ev;
			if (es < 0)
				ev = 0;
			else
				ev = 0.01*es*RH;
			double Vden = ev*100.0/RV/T*1000.;
			return Vden;
		}

		absorber::absorber(const atmoslayer &layer)
		{
			_init();
			setLayer(layer);
		}

		void absorber::setLayer(const atmoslayer &layer)
		{
			_p = &layer._p;
			_T = &layer._T;
			_dz = &layer._dz;
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
			_molnum = 0;
			_molecule = "";
			_layer = 0;
			_wvden = 0;
			_numConc = 0;
			_fr[0] = 0;
			_fr[1] = 0;
		}

		bool _findAbsorber
			(const std::string &molecule, double frequency, std::shared_ptr<absorber> &res)
		{
			absorber *newgas = 0;
			// frequency is not used yet, so the next line suppresses an error
			if (molecule == "O2" && frequency)
				newgas = new abs_O2;
			if (molecule == "N2")
				newgas = new abs_N2;
			if (molecule == "H2O")
				newgas = new abs_H2O;
			if (molecule == "COLLIDE")
				newgas = new collide;

			if (newgas)
			{
				std::shared_ptr<absorber> a(newgas);
				res = a;
				return true;
			}
			return false;
		}

	}; // end namespace atmos

}; // end namespace rtmath

