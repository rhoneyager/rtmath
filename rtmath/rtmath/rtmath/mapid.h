#pragma once
#include <cmath>
#include "defs.h"
#include "enums.h"
#include "Public_Domain/MurmurHash3.h"

namespace rtmath {
	// Used as an easy structure for mapping and function parameters
	class mapid
	{
	public:
		mapid(double mu, double mun, double phi, double phin)
		{
			this->mu = mu;
			this->mun = mun;
			this->phi = phi;
			this->phin = phin;
		}
		mapid () { mu=0; mun=0; phi=0; phin=0;}
		bool operator == (const mapid &rhs) const
		{
			if (this->mu != rhs.mu) return false;
			if (this->mun != rhs.mun) return false;
			if (this->phi != rhs.phi) return false;
			if (this->phin != rhs.phin) return false;
			return true;
		}
		bool operator != (const mapid &rhs) const
		{ return !(this->operator==(rhs)); }
		double mu, mun, phi, phin;
		inline HASH_t hash() const
		{
			HASH_t res;
			HASH(this, sizeof(*this), HASHSEED, &res);
			return res;
		}
		inline std::string print() const
		{
			std::ostringstream out;
			out << "mu: " << mu << " mun: " << mun << " phi: " << phi << " phin: " << phin;
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
