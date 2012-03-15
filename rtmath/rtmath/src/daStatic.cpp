#include "../rtmath/Stdafx.h"
#include <memory>
#define _USE_MATH_DEFINES
#include <math.h>
#include "../rtmath/da/damatrix.h"
#include "../rtmath/da/daStatic.h"

namespace rtmath {
	daStatic::daStatic()
	{
	}

	void daStatic::insert(const mapid &valmap, const std::shared_ptr<const matrixop> &val)
	{
		if (_srcs.count(valmap) == 0)
			_srcs[valmap] = val;
	}

	std::shared_ptr<const matrixop> daStatic::eval(const mapid &valmap) const
	{
		// First, check to see if this has already been calculated
		// If it is in the cache, return the cached value
		if (_eval_cache_enabled) 
			if (_eval_cache.count(valmap) > 0)
				return _eval_cache[valmap];

		// Perform the desired type of interpolation
		// TODO!
		throw rtmath::debug::xUnimplementedFunction();
		return nullptr;
		// 		if (_eval_cache_enabled)
			//_eval_cache[valmap] = res;
	}

}; // end rtmath
