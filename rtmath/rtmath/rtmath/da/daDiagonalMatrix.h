#pragma once
// Creates a diagonal matrix used in daLayer calculations
// This diagonal matrix has a pre-defined tau and is initialized to take 
// mu or mu_0 from a valmap and construct the diagonal matrix e^(-tau/mu(_0))

#include <memory>
#include "damatrix.h"
#include "../matrixop.h"
#include "../enums.h"

namespace rtmath {
	namespace valmap_selector {
		enum valmap_selector {
			MU,
			MUN
		};
	}; // end valmap_selector

	class daDiagonalMatrix : public damatrix
	{
	public:
		daDiagonalMatrix(double tau, valmap_selector::valmap_selector mumun);
		virtual ~daDiagonalMatrix();
		virtual std::shared_ptr<const matrixop> eval(const mapid &valmap) const;
	protected:
		double _tau;
		valmap_selector::valmap_selector _mumun;
		// Values may be cached. See damatrix.
	};

}; // end namespace rtmath

