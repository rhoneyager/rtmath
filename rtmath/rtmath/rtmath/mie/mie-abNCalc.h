#pragma once
#include <boost/shared_ptr.hpp>
#include <complex>
#include "../defs.h"

#pragma warning(push)
#pragma warning(disable: 4251) // dll-interface needed warning even though the affected member is private

namespace rtmath {
	namespace mie {
		class wnCalc;
		class AnCalc;

		class DLEXPORT abNCalc {
		public:
			// Need index of refraction and sizep
			abNCalc(const std::complex<double> &m, double sizep);
			void calc(unsigned int n, std::complex<double> &aRes, std::complex<double> &bRes) const;
		private:
			const std::complex<double> m;
			const double sizep;
			boost::shared_ptr<wnCalc> wn;
			boost::shared_ptr<AnCalc> an;
		};


	}
}

#pragma warning(pop)
