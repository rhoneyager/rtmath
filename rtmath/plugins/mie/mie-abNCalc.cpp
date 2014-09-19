#include "mie-abNCalc.h"
#include "mie-AnCalc.h"
#include "mie-wnCalc.h"

namespace rtmath {
	namespace plugins {
		namespace mie {

			abNCalc::abNCalc(const std::complex<double> &m, double sizep)
				: m(m), sizep(sizep)
			{
				using namespace std;
				an = boost::shared_ptr<AnCalc>(new AnCalc(sizep, m));
				wn = boost::shared_ptr<wnCalc>(new wnCalc(sizep));
			}

			void abNCalc::calc(unsigned int n, std::complex<double> &aRes, std::complex<double> &bRes) const
			{
				using namespace std;
				// Calculate an and bn in pairs because it's faster (duplicate steps)
				complex<double> aFirst;
				complex<double> bFirst;
				complex<double> _Wn = wn->calc(n);
				complex<double> _Wnm1 = wn->calc(n - 1);
				complex<double> _An = an->calc(n);
				aFirst = (_An / m) + complex<double>((double)n / sizep, 0.0);
				bFirst = (m*_An) + complex<double>((double)n / sizep, 0.0);

				aRes = ((aFirst * _Wn.real()) - _Wnm1.real()) / ( (aFirst * _Wn)-_Wnm1);
				bRes = ((bFirst * _Wn.real()) - _Wnm1.real()) / ( (bFirst * _Wn)-_Wnm1);
				std::cerr << "  abNCalc n " << n << " aFirst " << aFirst << " bFirst " << bFirst << std::endl;
				std::cerr << "          _Wn " << _Wn << " _Wnm1 " << _Wnm1 << " _An " << _An << std::endl;
				std::cerr << "          aFirst " << aFirst << " bFirst " << bFirst << std::endl;
				std::cerr << "          aRes " << aRes << " bRes " << bRes << std::endl;
			}


		}
	}
}
