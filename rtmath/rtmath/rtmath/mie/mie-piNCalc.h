#pragma once
#include <cstddef>
#include <vector>
#include "../defs.h"

#pragma warning(push)
#pragma warning(disable: 4251) // dll-interface needed warning even though the affected member is private

namespace rtmath {
	namespace mie {

		class DLEXPORT piNCalc {
		public:
			piNCalc(double mu);
			const double mu;
			double calc(size_t n) const;
		private:
			mutable std::vector<double> pin;
		};


	}
}

#pragma warning(pop)
