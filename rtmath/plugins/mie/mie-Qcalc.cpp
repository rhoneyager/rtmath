#include "mie-Qcalc.h"
#include "mie-abNCalc.h"

namespace rtmath {
	namespace plugins {
		namespace mie {

			Qcalc::Qcalc(const std::complex<double> &m, double tolerance, double atol)
				: _m(m), _tolerance(tolerance), _atol(atol) {}

			void Qcalc::calc(double sizep, double &Qext, double &Qsca, double &Qabs, double &Qbk, double &g) const
			{
				using namespace std;
				// TODO: save previous results for faster future runs!!!!!!!
				// Construct the abNcalc class
				abNCalc abn(_m, sizep);
				// The summations will be performed to within the tolerance value
				// That is, the summing stops once the tolerance is reached
				// Q_p are the previous values
				double Qe = 0, Qep, Qepar;
				double Qs = 0, Qsp, Qspar;
				double Qa = 0;
				double Qb = 0;

				complex<double> Qbp, Qbpar;
				double gpar = 0;

				// Define memory for an and bn
				complex<double> an, anp;
				complex<double> bn, bnp;
				complex<double> suma, sumb;
				abn.calc(1, an, bn);
				// Wiscombe 1979
				double nc = 0;
				if (sizep<8) nc = sizep + 4.*pow(sizep, 1. / 3.) + 1.;
				else
					nc = sizep + 4.05*pow(sizep, 1. / 3.) + 2.; // Candidate bounding formula
				for (unsigned int n = 1; n < (unsigned int)(nc + 1); n++)
				{
					// Set previous values for iteration
					Qep = Qe;
					Qsp = Qs;
					// Qbp is preserved

					// Do iteration of the sum
					abn.calc(n + 1, anp, bnp);

					Qepar = (2.0*n + 1.0) * (an.real() + bn.real());
					Qspar = (2.0*n + 1.0) * ((an * conj(an)) + (bn * conj(bn))).real();
					//	std::abs(an) * std::abs(an)) + (std::abs(bn) * std::abs(bn)) );
					Qbpar = complex<double>((2.0*n + 1.0)*pow(-1.0, n), 0.0) * (an - bn);
					suma = an*conj(anp) + bn*conj(bnp);
					sumb = an*conj(bn);

					Qe += Qepar;
					Qs += Qspar;
					Qbp += Qbpar;
					gpar += ((n*(n + 2.0) / (n + 1.0)) * suma.real()) + (((2.0*n + 1) / (n*n + n))* sumb.real());
					an = anp;
					bn = bnp;
					// At least the n=1 terms will pass
					// Do another tolerance check against too small terms messing up sum
					//if (abs(Qepar) < _atol || abs(Qspar) < _atol ) break;
					//if (abs(Qbpar.real()) < _atol) break;
					//if (abs(Qbpar.imag()) < _atol) break;	
					// Note the inversion of div and inequalities to make this work
					//if ( (Qep / Qe > _tolerance) && (Qsp/Qs >_tolerance) ) break;
				}
				Qe *= 2.0 / (sizep*sizep);
				Qs *= 2.0 / (sizep*sizep);
				Qa = Qe - Qs; // Will be negative here.

				Qb = (Qbp * conj(Qbp)).real();
				Qb *= 1.0 / (sizep*sizep);

				// Set final values
				Qext = Qe;
				Qsca = Qs;
				Qabs = Qa;
				Qbk = Qb;

				// Now, to calculate g
				// I'm doing this here because the summations are already in place, and g is really a waste of a class
				// abncalc already holds the necessary values
				gpar *= 4.0 / (sizep*sizep);
				g = gpar / Qsca;
				return;
			}

		}
	}
}
