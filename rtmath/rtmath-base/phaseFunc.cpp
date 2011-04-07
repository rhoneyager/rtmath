#include "phaseFunc.h"
#include <complex>
namespace rtmath {

phaseFunc::phaseFunc(void)
{
}


phaseFunc::~phaseFunc(void)
{
}


scattMatrix::scattMatrix(void)
{
}

scattMatrix::~scattMatrix(void)
{
}

void scattMatrix::_genMuellerMatrix(double Snn[4][4], std::complex<double> Sn[4])
{
	// Do the upper left quad in a loop
	for (unsigned int i=0;i<4;i++)
	{
		Snn[0][0] += 0.5 * (abs(Sn[i])*abs(Sn[i]));
		Snn[0][1] += 0.5 * pow(-1.0,(int)i) * (abs(Sn[i])*abs(Sn[i]));
		if (i == 2 || i == 3)
		{
			Snn[1][1] -= 0.5 * (abs(Sn[i])*abs(Sn[i]));
		} else {
			Snn[1][1] += 0.5 * (abs(Sn[i])*abs(Sn[i]));
		}
		if (i == 1 || i == 2)
		{
			Snn[1][0] += 0.5 * (abs(Sn[i])*abs(Sn[i]));
		} else {
			Snn[1][0] -= 0.5 * (abs(Sn[i])*abs(Sn[i]));
		}
	}
	// Do the rest this way
	std::complex<double> scratch;
	scratch = ( (Sn[1] * (conj(Sn[2]))) + (Sn[0] * (conj(Sn[3])) )).real();
	Snn[0][2] = scratch.real();
	scratch = ( (Sn[1] * (conj(Sn[2]))) - (Sn[0] * (conj(Sn[3])) )).imag();
	Snn[0][3] = scratch.real();

	scratch = ( (Sn[1] * (conj(Sn[2]))) - (Sn[0] * (conj(Sn[3])) )).real();
	Snn[1][2] = scratch.real();
	scratch = ( (Sn[1] * (conj(Sn[2]))) + (Sn[0] * (conj(Sn[3])) )).imag();
	Snn[1][3] = scratch.real();

	scratch = ( (Sn[1] * (conj(Sn[3]))) + (Sn[0] * (conj(Sn[2])) )).real();
	Snn[2][0] = scratch.real();
	scratch = ( (Sn[1] * (conj(Sn[3]))) - (Sn[0] * (conj(Sn[2])) )).real();
	Snn[2][1] = scratch.real();

	scratch = ( (Sn[0] * (conj(Sn[1]))) + (Sn[2] * (conj(Sn[3])) )).real();
	Snn[2][2] = scratch.real();
	scratch = ( (Sn[1] * (conj(Sn[0]))) + (Sn[3] * (conj(Sn[2])) )).imag();
	Snn[2][3] = scratch.real();

	scratch = ( (conj(Sn[1]) * (Sn[3])) + (conj(Sn[2]) * (Sn[0]) )).imag();
	Snn[3][0] = scratch.real();
	scratch = ( (conj(Sn[1]) * (Sn[3])) - (conj(Sn[2]) * (Sn[0]) )).imag();
	Snn[3][1] = scratch.real();

	scratch = ( (Sn[0] * (conj(Sn[1]))) - (Sn[2] * (conj(Sn[3])) )).imag();
	Snn[3][2] = scratch.real();
	scratch = ( (Sn[0] * (conj(Sn[1]))) - (Sn[2] * (conj(Sn[3])) )).real();
	Snn[3][3] = scratch.real();
}

}; // end rtmath

