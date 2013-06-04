#include "../rtmath/Stdafx.h"
#include "../rtmath/ddscat/mtab.h"
#include "../rtmath/error/error.h"

namespace rtmath {
	namespace ddscat {

		mtab::~mtab()
		{
		}

		//mtab::mtab(void (*provider)(double, double, std::complex<double>&) )
		mtab::mtab(
			std::function<void(double,double,std::complex<double>&)> provider)
		{
			_provider = provider;
			_T = 0;
			_f = 0;
		}

		void mtab::eval(std::complex<double> &res) const
		{
			if (_f == 0 || _T == 0)
				throw rtmath::debug::xBadInput("mtab eval has invalid freq / temp");

			_provider(_f, _T, res);
		}

		void mtab::write(const std::string &filename) const
		{
			std::ofstream out(filename.c_str());
			write(out);
		}

		void mtab::write(std::ostream &out) const
		{
			std::complex<double> ref;
			eval(ref);

			using namespace std;
			out.setf( ios::scientific, ios::floatfield);
			out.precision(7);
			out << " m = " << ref.real() << " + " << (-1.0 *ref.imag()) << " i" << endl;
			out << " 1 2 3 0 0 = columns for wave, Re(n), Im(n), eps1, eps2" << endl;
			out << " LAMBDA  Re(N)   Im(N)" << endl;
			out << " 0.000001    " << ref.real() << "      " << (-1.0*ref.imag()) << endl;
			out << " 1.000000    " << ref.real() << "      " << (-1.0*ref.imag()) << endl;
			out << " 100000.0    " << ref.real() << "      " << (-1.0*ref.imag()) << endl;
		}
	}
}

namespace std {
	ostream & operator<<(ostream &stream, const rtmath::ddscat::mtab &ob)
	{
		ob.write(stream);
		return stream;
	}

}


