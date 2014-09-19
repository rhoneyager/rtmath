#pragma once
#include <complex>

#pragma warning(push)
#pragma warning(disable: 4251) // dll-interface needed warning even though the affected member is private

namespace rtmath {
	namespace plugins {
		namespace mie {

			class  Qcalc
			{
			public:
				Qcalc(const std::complex<double> &m, double tolerance = 1.01, double atol = 1.e-9);
				void calc(double sizep, double &Qext, double &Qsca, double &Qabs, double &Qbk, double &g) const;
			private:
				const std::complex<double> _m;
				const double _tolerance;
				const double _atol;
			};
		}
	}
}

#pragma warning(pop)
