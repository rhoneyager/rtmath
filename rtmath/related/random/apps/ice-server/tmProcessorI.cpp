#include <iostream>
#include "../../src/headers/tmatrix.h"
#include "tmProcessorI.h"
#include <src/tmatrixBase.h>
#include <Ryan_Serialization/serialization.h>

namespace tmatrix
{
	tmProcessorI::tmProcessorI() {}

	std::string tmProcessorI::submit(
		const std::string &request, const Ice::Current &c)
	{
		IceUtil::Mutex::Lock lock(m);

		using namespace std;
		using namespace tmatrix;
		try
		{
			queue::TM tm;
			Ryan_Serialization::readString<queue::TM>(tm, request);

			tm.calc();

			string result;
			Ryan_Serialization::writeString<queue::TM>(tm, result);
			return result;
		}
		catch (std::exception &e)
		{
			std::cerr << e.what() << endl;
		}
		return std::string();
	}


}
