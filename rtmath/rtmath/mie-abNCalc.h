#pragma once
#include "mie-AnCalc.h"
#include "mie-wnCalc.h"
#include<complex>

namespace mie {

class abNCalc {
public:
    // Need index of refraction and x
    abNCalc(const std::complex<double> &m, double x);
    ~abNCalc();
    void calc(unsigned int n, std::complex<double> &aRes, std::complex<double> &bRes);
private:
    std::complex<double> m;
    double x;
    wnCalc *wn;
    AnCalc *an;
};


}; //end mie

