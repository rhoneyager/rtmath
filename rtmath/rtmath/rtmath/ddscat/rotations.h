#pragma once

#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <set>
#include <vector>
#include <complex>
#include <boost/tokenizer.hpp>
#include "ddpar.h"

namespace rtmath {
	namespace ddscat {

		class rotationsBase
		{
		public:
			rotationsBase()
				:
				_bMin(0), _bMax(360), _bN(6),
				_tMin(0), _tMax(90), _tN(6),
				_pMin(0), _pMax(180), _pN(6)
				{ }
			virtual ~rotationsBase();
		protected:
			double _bMin, _bMax;
			double _tMin, _tMax;
			double _pMin, _pMax;
			size_t _bN, _tN, _pN;
		};

		class rotations : public rotationsBase
		{
		public:
			rotations();
			rotations(const ddPar &src);
			rotations(double bMin, double bMax, size_t bN,
				double tMin, double tMax, size_t tN,
				double pMin, double pMax, size_t pN);
			virtual ~rotations();
			double bMin() const { return _bMin; }
			double bMax() const { return _bMax; }
			double tMin() const { return _tMin; }
			double tMax() const { return _tMax; }
			double pMin() const { return _pMin; }
			double pMax() const { return _pMax; }
		};

	}
}


