#pragma once
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <complex>
#include "../matrixop.h"
#include "../interpolatable.h"
#include "../phaseFunc.h"
#include "../ddscat/ddScattMatrix.h"
#include "../ddscat/ddOutputSingle.h"
#include "../da/daStatic.h"
#include "../coords.h"
	
namespace mie {

	class ddOutputMie : public rtmath::ddscat::ddOutputSingle
	{
		// Class contains the output of a single ddscat file
		// Doesn't quite inherit from daStatic. The files loaded by the ddOutput class
		// contain information on the scattering and emission matrices, so they are logically
		// two separate entries.
		// Note: ensemble providers inherit from this!
	public:
		// Constructors need to completely initialize, as I have const ddOutputSingle
		// shared pointers.
		// Read from ddscat file
		ddOutputMie(double freq, double psize, double span = 5.0);

		virtual ~ddOutputMie();

	private:
		void _init();

	public: // Made public for now so ensembles work. May just make that a friend class.
		ddOutputMie();
		//virtual void _insert(std::shared_ptr<const rtmath::ddscat::ddScattMatrix> &obj);
	};

}

