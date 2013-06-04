#pragma once

#include<vector>
#include<complex>
#include "../defs.h"

#pragma warning(push)
#pragma warning(disable: 4251) // dll-interface needed warning even though the affected member is private

namespace rtmath {
	namespace mie {

		class DLEXPORT AnCalc
		{
		public:
			std::complex<double> calc(size_t n) const;
			AnCalc(double sizep, const std::complex<double> &m);
		private:
			mutable std::vector< std::complex<double> > An;
			const double sizep;
			const std::complex<double> m;
		};

	}
}

#pragma warning(pop)
