#pragma once
#include "../defs.h"

namespace rtmath
{
	namespace mie
	{
		struct mieBase;
		class mieParams;
		class mieCalc;
		class mieAngleRes;
	}
}

namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void DLEXPORT serialize(Archive & ar, rtmath::mie::mieParams & g, const unsigned int version);

		template <class Archive>
		void DLEXPORT serialize(Archive & ar, rtmath::mie::mieCalc & g, const unsigned int version);

		template <class Archive>
		void DLEXPORT serialize(Archive & ar, rtmath::mie::mieAngleRes & g, const unsigned int version);

	}
}

