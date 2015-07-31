#include "Stdafx-core.h"
#include <cmath>
#include <map>
#include "../rtmath/psd.h"
#include "../rtmath/units.h"
#include "../rtmath/zeros.h"
#include "../rtmath/quadrature.h"
#include <Ryan_Debug/error.h>

namespace rtmath
{
	namespace psd
	{
		namespace implementations {

			//namespace tag {
				namespace SekhonSrivastava1970 {
					double N0(double R) { return 2500. * pow(R,-0.94); }
					double lambda(double R) { return 22.9 * pow(R,-0.45); }
					double Nd(double R, double Dmelt) { return N0(R) * exp(-lambda(R) * Dmelt); }
					double NdI(double R) { return N0(R) / lambda(R); }
					double median(double R) { return log(2.) / lambda(R); }
				}

				namespace GunnMarshall1958 {
					double N0(double R) { return 3800. * pow(R,-0.87); }
					double lambda(double R) { return 25.5 * pow(R,-0.48); }
					double Nd(double R, double Dmelt) { return N0(R) * exp(-lambda(R) * Dmelt); }
					double NdI(double R) { return N0(R) / lambda(R); }
					double median(double R) { return log(2.) / lambda(R); }
				}

				namespace MarshallPalmer1948 {
					double N0(double) { return 8000; }
					double N0() { return 8000; }
					double lambda(double R) { return 41 * pow(R,-0.21); }
					double Nd(double R, double Dmelt) { return N0(R) * exp(-lambda(R) * Dmelt); }
					double NdI(double R) { return N0(R) / lambda(R); }
					double median(double R) { return log(2.) / lambda(R); }
				}

			//}

		}
	}
}

