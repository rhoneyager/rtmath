#pragma once
#include<complex>
#include<map>

#pragma warning(push)
#pragma warning(disable: 4251) // dll-interface needed warning even though the affected member is private
namespace rtmath {
	namespace plugins {
		namespace mie {

			class wnCalc
			{
			public:
				wnCalc(double sizep);
				std::complex<double> calc(int n) const;
			private:
				mutable std::map<int, std::complex<double> > _Wn;
				const double _sizep;
			};
		}
	}
}
#pragma warning(pop)