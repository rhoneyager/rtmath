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

		double absorber::_wvtofreq(double nu)
		{
			double f = nu * 2.99792458e10/1.e9; // Convert cm^-1 to GHz
			return f;
		}

		double absorber::_freqtowv(double f)
		{
			double nu = f*1.e9/2.997925e10; // convert GHz to cm^-1
			return nu;
		}

		double absorber::_rho(double P, double T, double ew)
		{
			// T is in K, P is in hPa, ew is in hPa
			// rho is in number of molecules / m^3
			const double Rd = 287.058; // J/kg*K
			const double Rv = 461.5; // J/kg*K
			const double Na = 6.02214129e23; // molecules / mole
			double rho;
			rho = ew/(Rv*T);
			rho += (P-ew)/(Rd*T);
			rho *= 100; // since pressures in hPa. Convert to Pa
			rho *= Na; // moles to molecules
			return rho; // molecules per cubic meter
		}

		double absorber::_ewsat(double P, double T)
		{
			// T is in K, P is in hPa
			// Result is in hPa
			double tc = T - 273.15;
			double e = (1.0007+3.46e-6*P)*6.1121;
			e += exp(17.502*tc/(240.97+tc));
			return e;
		}

		double absorber::_Vden(double T, double RH)
		{
			const double RV = 461.5;
			const double es = 6.1365*exp(17.502*(T-273.15)/(T-32.18));
			double ev;
			if (es < 0)
				ev = 0;
			else
				ev = 0.01*es*RH;
			// TODO: verify for clarity
			double Vden = ev*1.e5/(RV*T);
			return Vden;
		}

		double absorber::_RH(double T, double rhowat)
		{
			const double RV = 461.5;
			const double es = 6.1365*exp(17.502*(T-273.15)/(T-32.18));
			double ev = rhowat * RV * T / 1.e5;
			double RH = 100.0 * ev / es;
			return RH;
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

		bool absorber::_findAbsorber
			(const std::string &molecule, std::shared_ptr<absorber> &res, double frequency)
		{
			absorber *newgas = nullptr;
			// frequency is not used yet, so the next line suppresses a warning
			frequency++;
			if (molecule == "O2")
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

