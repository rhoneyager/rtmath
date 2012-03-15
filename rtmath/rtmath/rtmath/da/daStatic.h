#pragma once

// This class provides a damatrix that is interpolatable and is based on 
// a series of ddscat entries / constant matrixops for a given rotation.
// It is of immense use in calculating the phase function for discrete 
// quadrature angles.

#include <memory>
#include "../matrixop.h"
#include "damatrix.h"
#include "../phaseFunc.h"
#include "../enums.h" // provides rtselec

namespace rtmath {
	class daStatic : public damatrix
	{
		public:
			daStatic();
			virtual ~daStatic() {}
			virtual void insert(const mapid &valmap, const std::shared_ptr<const matrixop> &val);
			// Evaluation here involves interpolation of known results
			// Results are cached for speed
			virtual std::shared_ptr<const matrixop> eval(const mapid &valmap) const;
		private:
			std::unordered_map<mapid, std::shared_ptr<const matrixop>, boost::hash<rtmath::mapid> > _srcs;
	};

}; // end rtmath
