#pragma once
#include <src/tmatrixBase.h>
#include <IceUtil/Mutex.h>

namespace tmatrix
{
	class tmProcessorI : public tmProcessor
	{
	public:
		tmProcessorI();
		virtual std::string submit(const std::string &request, 
			const Ice::Current& = Ice::Current()) override;
		//virtual void destroy(const Ice::Current& = Ice::Current()) override;
	private:
		IceUtil::Mutex m;
	};

}
