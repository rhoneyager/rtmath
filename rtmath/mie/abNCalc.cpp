#include "abNCalc.h"

namespace mie {

abNCalc::abNCalc(const std::complex<double> &m, double x)
{
    using namespace std;
    this->m = m;
    this->x = x;
    this->an = new AnCalc(x,m);
    this->wn = new wnCalc(x);
}

abNCalc::~abNCalc()
{
    delete this->an;
    delete this->wn;
}

void abNCalc::calc(unsigned int n, std::complex<double> &aRes, std::complex<double> &bRes)
{
    using namespace std;
    // Calculate an and bn in pairs because it's faster (duplicate steps)
    complex<double> aFirst(0.0,0.0);
    complex<double> bFirst(0.0,0.0);
    complex<double> _Wn = wn->calc(n);
    complex<double> _Wnm1 = wn->calc(n-1);
    complex<double> _An = an->calc(n);
    aFirst = (_An / m) + complex<double>( (double) n / x,0.0);
    bFirst = (m*_An) + complex<double>( (double) n / x,0.0);
    
    aRes = ((aFirst) * (_Wn.real() - _Wnm1.real())) / (aFirst * (_Wn - _Wnm1));
    bRes = ((bFirst) * (_Wn.real() - _Wnm1.real())) / (bFirst * (_Wn - _Wnm1));
    return;
}


}; // end mie

