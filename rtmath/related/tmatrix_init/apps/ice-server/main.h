#pragma once
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <IceBox/IceBox.h>
#include <IceStorm/IceStorm.h>
#include <Freeze/Freeze.h>

#if defined(_WIN32)
#   define EXPORTING __declspec(dllexport)
#else
#   define EXPORTING /**/
#endif

/// Provides Ice::Service methods for starting and stopping the server. Also provides common objects to the rest of the program.
class EXPORTING app : public IceBox::Service
{
public:
	static app* instance();
	virtual void start(	const std::string& name,
						const Ice::CommunicatorPtr& communicator,
						const Ice::StringSeq& args);
	virtual void stop();

public:
	Ice::ObjectAdapterPtr _adapter;
	//Freeze::EvictorPtr evictor;
	Ice::LoggerPtr logger;
	bool trace;
	Ice::CommunicatorPtr communicator;
	//IceStorm::TopicManagerPrx topicManager;
	Ice::Identity id;
private:
	//appMaproom();
};


