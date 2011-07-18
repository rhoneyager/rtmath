#pragma once
#include <memory>
#include "damatrix.h"
#include "matrixop.h"

// Basic override for the standard damatrix, which allows the user to set a provider function 
// for the matrixop when evaluating

// Layers have their own special initial matrix calculation class, which is similar to 
// this prototype.

namespace rtmath {

	typedef matrixop (*damatrix_provider_function)(const mapid &valmap);

	class damatrix_override : public damatrix
	{
	public:
		damatrix_override(damatrix_provider_function provider) : damatrix()
		{
			_providerFunction = provider;
		}
		virtual ~damatrix_override() {}
		virtual std::shared_ptr<matrixop> eval(const mapid &valmap)
		{
			std::shared_ptr<matrixop> res( new matrixop(_providerFunction(valmap)));
			return res;
		}

	protected:
		damatrix_provider_function _providerFunction;				// The function that populates the matrixop upon eval
	};

}; // namespace rtmath
