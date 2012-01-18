#include "../rtmath/Stdafx.h"
#include <memory>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <sstream>
#include "../rtmath/atmos.h"
#include "../rtmath/atmoslayer.h"
#include "../rtmath/absorb.h"

namespace rtmath {
	
	namespace atmos {

		void atmoslayer::_init()
		{
			_p = 0;
			_T = 0;
			_dz = 0;
		}

		atmoslayer::atmoslayer(const atmoslayer &rhs)
		{
			_init();
			*this = rhs; // Invoke the assignment operator
		}

		atmoslayer & atmoslayer::operator= (const atmoslayer & rhs)
		{
			// First, check for self-assignment
			if (this == &rhs) return *this;
			// Now, duplicate _p, _T, _dz
			_p = rhs._p;
			_T = rhs._T;
			_dz = rhs._dz;
			// Duplicate the absorbers
			// Slightly harder, as it is a set of unique_ptr
			std::set<std::shared_ptr<absorber> >::iterator it; // Note: iterator cannot be const

			for (it = rhs.absorbers.begin(); it != rhs.absorbers.end(); it++)
			{
				// Take each element and invoke its clone method to do a deep copy
				// and add this to the new set. Also, make sure that the absorber
				// is bound to this class instance, not the previous one.
				std::shared_ptr<absorber> np( (*it)->clone() );
				np->setLayer(*this);
				absorbers.insert( np );
			}
			return *this;
		}

		atmoslayer::atmoslayer(double p, double T, double dz)
		{
			_init();
			_p = p;
			_T = T;
			_dz = dz;
		}

		double atmoslayer::tau(double f) const
		{
			double res = 0;
			if (this->dz() == 0) return 0; // For TOA layer
			std::set<std::shared_ptr<absorber> >::const_iterator it;
			for (it = absorbers.begin(); it != absorbers.end(); it++)
			{
				double t = (*it)->deltaTau(f);
				res += t;
			}
			return res;
		}

		//atmoslayer* atmoslayer::clone() const
		//{
		//}


	}; // end namespace atmos

}; // end namespace rtmath

