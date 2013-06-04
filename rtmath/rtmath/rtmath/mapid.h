#pragma once
#include <cmath>
#include <sstream>
#include "defs.h"
#include "enums.h"
#include "common_templates.h"
#include "Public_Domain/MurmurHash3.h"

namespace rtmath {
	// Used as an easy structure for mapping and function parameters
	class mapid
	{
	public:
		friend struct std::less<rtmath::mapid>;
		mapid(double mu, double mun, double phi, double phin, double f = 0)
		{
			this->mu = mu;
			this->mun = mun;
			this->phi = phi;
			this->phin = phin;
			this->f = f;
		}
		mapid () { mu=0; mun=0; phi=0; phin=0; f = 0;}
		bool operator == (const mapid &rhs) const
		{
			if (this->f != rhs.f) return false;
			if (this->mu != rhs.mu) return false;
			if (this->mun != rhs.mun) return false;
			if (this->phi != rhs.phi) return false;
			if (this->phin != rhs.phin) return false;
			return true;
		}
		bool operator != (const mapid &rhs) const
		{ return !(this->operator==(rhs)); }
		double mu, mun, phi, phin, f;
		inline std::string print() const
		{
			std::ostringstream out;
			out << "f: " << f << "mu: " << mu << " mun: " << mun << " phi: " << phi << " phin: " << phin;
			std::string res = out.str();
			return res;
		}
		inline double toCosAlpha(rtselec::rtselec RT) const
		{
			double cosa = 0;
			cosa = sqrt(1.0 - (mu * mu) ) * sqrt(1.0 - (mun * mun) ) * cos(phi - phin);
			switch (RT)
			{
			case rtselec::R:
				cosa -= mu*mun;
				break;
			case rtselec::T:
				cosa += mu*mun;
				break;
			}
			return cosa;
		}
		inline double toAlpha(rtselec::rtselec RT) const
		{
			return acos( toCosAlpha(RT) );
		}
	};

	
}; // end namespace rtmath

// Supporting code to allow an unordered map of mapid (for damatrix)
// Using standard namespace for C++11
namespace std {


	template <> struct less<rtmath::mapid >
	{
		bool operator() (const rtmath::mapid &lhs, const rtmath::mapid &rhs) const
		{
			// Check f, mu, mun, phi, phin
			if (lhs.f != rhs.f) return lhs.f < rhs.f;
			if (lhs.mu != rhs.mu) return lhs.mu < rhs.mu;
			if (lhs.mun != rhs.mun) return lhs.mun < rhs.mun;
			if (lhs.phi != rhs.phi) return lhs.phi < rhs.phi;
			if (lhs.phin != rhs.phin) return lhs.phin < rhs.phin;

			return false;
		}
	};
}; // end namespace std

