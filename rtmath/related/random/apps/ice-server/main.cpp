#include <Ice/Ice.h>
#include <IceBox/IceBox.h>
#include <IceGrid/IceGrid.h>
#include <IceStorm/IceStorm.h>
#include <IceUtil/IceUtil.h>
#include <Freeze/Freeze.h>
#include <iostream>
#include "main.h"
#include "tmProcessorI.h"

using namespace std;

/// Useful function to get public program-wide objects (like the logger and communicator)
app* app::instance()
{
	std::cerr << "Calling app::instance\n";
	static app *m = nullptr;
	if (!m) m = new app;
	return m;
}

extern "C" {
	/// Entry point when executed as a DLL or .so from IceGrid
	EXPORTING IceBox::Service*
	create(Ice::CommunicatorPtr communicator)
	{
		std::cerr << "Calling create\n";
		return app::instance();
	}
}

/*
int main(int argc, char** argv)
{
	std::cerr << "Calling main\n";
	// Set the 'BuildId' property displayed in the IceGridAdmin GUI
    Ice::InitializationData initData;
    initData.properties = Ice::createProperties(argc,argv);
    initData.properties->setProperty("BuildId", string("Ice ") + ICE_STRING_VERSION);

	// Load config.server by default
	//if (argc == 1)
	//	initData.properties->load("config.server"); // A nice default
	
	std::cerr << "argc " << argc << std::endl;
	std::cerr << "argv ";
	for (int i=0;i<argc;i++)
		std::cerr << argv[i] << " ";
	std::cerr << std::endl;

	app* a = app::instance();

	/// Construct communicator
	Ice::CommunicatorPtr communicator = Ice::initialize(argc,argv, initData);
	Ice::StringSeq args = Ice::argsToStringSeq(argc, argv);

	a->start("TmatrixServer", communicator, args);

	communicator->waitForShutdown();
	a->stop();
	
    return 0;
}
*/

/**
 * \brief The real starting point for most program execution
 *
 * This function performs several important tasks. In order:
 * - It creates the object adapter. This functions as the 'listening' part of the server.
 * - It contacts the topicManager (in the IceStorm proxy). This allows for announcements to clients to be made.
 * - It connects to the evictor and creates to evictor object factory. The evictor is the 
 *   database that stores all of the objects when they are not in active use. Its object factory provides the 
 *   code to initialize new objects.
 * - It registers the various object factories and servant locators with the Ice::Communicator.
 * - It creates / reloads the object factories.
 * - It starts any necessary timers.
 * - It instructs the adapter to start listening for client connections.
 **/
void app::start(const std::string& name,
						const Ice::CommunicatorPtr& communicator,
						const Ice::StringSeq& args)
{
	// Arguments determine if this is a dispatcher or a renderer process
	trace = communicator->getProperties()->getPropertyAsIntWithDefault("Server.Trace", 0) != 0;
	logger = communicator->getLogger();
	try {

		// Get properties and create the object adapter
		this->communicator = communicator;
		Ice::PropertiesPtr properties = communicator->getProperties();
		id = communicator->stringToIdentity(properties->getProperty("Identity"));
		
		//trace = communicator->getProperties()->getPropertyAsIntWithDefault("Server.Trace", 0) != 0;
		//logger = communicator->getLogger();

		logger->trace("info", std::string("Starting tmatrix server with name: ").append(id.name) );
		_adapter = communicator->createObjectAdapter(name);

		// Start listening for client connections
		logger->trace("info", std::string("Activating adapter"));

		tmatrix::tmProcessorPtr proc = new tmatrix::tmProcessorI;
		_adapter->add(proc, id);
		_adapter->activate();
	}
	catch (std::exception &e)
	{
		logger->trace("error", std::string(e.what()) );
		exit(1);
	}
	catch (...)
	{
		exit(1);
	}
}

// Called when the dispatcher is to be closed. Logs the termination and destroys _adapter.
void app::stop()
{
	logger->trace("info", std::string("Stopping tmatrix server ").append(id.name) );

	_adapter->destroy();

	//communicator->shutdown();
	//communicator->waitForShutdown();
}
