#include "Stdafx-mie.h"
#include "../rtmath/mie/mie-abNCalc.h"
#include "../rtmath/mie/mie-AnCalc.h"
#include "../rtmath/mie/mie-wnCalc.h"

namespace rtmath {
	namespace mie {

		abNCalc::abNCalc(const std::complex<double> &m, double sizep)
			: m(m), sizep(sizep)
		{
			using namespace std;
			an = boost::shared_ptr<AnCalc>(new AnCalc(sizep,m));
			wn = boost::shared_ptr<wnCalc>(new wnCalc(sizep));
		}

		void abNCalc::calc(unsigned int n, std::complex<double> &aRes, std::complex<double> &bRes) const
		{
			using namespace std;
			// Calculate an and bn in pairs because it's faster (duplicate steps)
			complex<double> aFirst;
			complex<double> bFirst;
			complex<double> _Wn = wn->calc(n);
			complex<double> _Wnm1 = wn->calc(n-1);
			complex<double> _An = an->calc(n);
			aFirst = (_An / m) + complex<double>( (double) n / sizep,0.0);
			bFirst = (m*_An) + complex<double>( (double) n / sizep,0.0);

			aRes = ((aFirst) * (_Wn.real()) - _Wnm1.real()) / (aFirst * (_Wn) - _Wnm1);
			bRes = ((bFirst) * (_Wn.real()) - _Wnm1.real()) / (bFirst * (_Wn) - _Wnm1);
		}


	}
}
