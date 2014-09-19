#include "../../rtmath/rtmath/phaseFunc.h"
#include "mie-Scalc.h"
#include "mie-abNCalc.h"
#include "mie-piNCalc.h"
#include "mie-tauNCalc.h"

#include <cmath>

namespace rtmath {
	namespace plugins {
		namespace mie {

			Scalc::Scalc(const std::complex<double> &m, double sizep, double tol, double atol)
				: _tolerance(tol), _atol(atol), _m(m), sizep(sizep) {}

			void Scalc::calc(double mu, Eigen::Matrix2cd& Sn, Eigen::Matrix4d& Snn) const
			{
				// This code is based on the Qcalc code
				// TODO: save previous results for faster future runs!!!!!!!
				// Construct the abNcalc class
				abNCalc abn(_m, sizep);

				// The summations will be performed to within the tolerance value
				// That is, the summing stops once the tolerance is reached
				// Q_p are the previous values
				piNCalc piN(mu);
				tauNCalc tauN(mu);

				// Use Sa, Sb, Ssq that are provided
				std::complex<double> Sap, Sapar;
				std::complex<double> Sbp, Sbpar;

				size_t nmax = (size_t)(sizep + 4.0*pow(sizep, 1. / 3.) + 2.0) + 1;
				//for (size_t n=1; n < 400; n++)
				for (size_t n = 1; n < nmax; n++)
				{
					// Set previous values for iteration
					Sap = Sn(1, 1);
					Sbp = Sn(0, 0);
					// Define memory for an and bn
					std::complex<double> an;
					std::complex<double> bn;
					double pinRes;
					double taunRes;
					// Do iteration of the sum
					abn.calc((unsigned int)n, an, bn);
					pinRes = piN.calc(n);
					taunRes = tauN.calc(n);

					Sapar = an*pinRes + bn*taunRes;
					Sbpar = an*taunRes + bn*pinRes;
					Sapar *= (2.0*n + 1) / (n*n + n);
					Sbpar *= (2.0*n + 1) / (n*n + n);
					// Do another tolerance check against too small terms messing up sum
					if (abs(Sapar) < _atol || abs(Sbpar) < _atol) break;
					Sn(1, 1) += Sapar;
					Sn(0, 0) += Sbpar;
					// Note the inversion of div and inequalities to make this work
					//if ( (Qep / Qe > _tolerance) && (Qsp/Qs >_tolerance) ) break;
				}

				rtmath::phaseFuncs::muellerBH(Sn, Snn);
				// Ssq is now Snn[0][0]
				//Ssq = 0.5 * (abs(*Sa)*abs(*Sa) + abs(*Sb) * abs(*Sb));

			}
		}
	}
}
