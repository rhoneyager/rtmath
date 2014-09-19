#pragma once
#include <complex>
#include <Eigen/Core>

#pragma warning(push)
#pragma warning(disable: 4251) // dll-interface needed warning even though the affected member is private

namespace rtmath {
	namespace plugins {
		namespace mie {

			class Scalc
			{
			public:
				Scalc(const std::complex<double> &m, double sizep, double tol = 1.01, double atol = 1.e-8);
				void calc(double mu, Eigen::Matrix2cd& Sn, Eigen::Matrix4d& Snn) const;
			private:
				const std::complex<double> _m;
				const double sizep;
				const double _tolerance;
				const double _atol;
			};
		}
	}
}

#pragma warning(pop)
