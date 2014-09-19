#pragma once
#include <boost/shared_ptr.hpp>

#pragma warning(push)
#pragma warning(disable: 4251) // dll-interface needed warning even though the affected member is private
namespace rtmath
{
	namespace plugins {
		namespace mie {
			class piNCalc;
			class tauNCalc {
			public:
				tauNCalc(double mu);
				double calc(size_t n) const;
			private:
				const double mu;
				boost::shared_ptr<piNCalc> pin;
			};

		}
	}
}

#pragma warning(pop)
