#include <Ice/Ice.h>
#include <Ice/Application.h>
#include <IceBox/IceBox.h>
#include <IceGrid/IceGrid.h>
#include <IceStorm/IceStorm.h>
#include <IceUtil/IceUtil.h>
#include <Freeze/Freeze.h>
#include <iostream>
//#include "main.h"
#include "../../src/headers/tmatrix.h"
#include <src/tmatrixBase.h>
#include <boost/shared_ptr.hpp>
#include <Ryan_Serialization/serialization.h>
//#include <Ryan-Debug/debug.h>
//#include <tmatrixBase.h>
//#include 

using namespace std;
using namespace tmatrix;

class TMclient : public Ice::Application
{
public:
	virtual int run(int argc, char** argv)
	{
		try {
			Ice::ObjectPrx proxy = 
				communicator()->stringToProxy("tmProcessor");
			tmProcessorPrx worker = 
				tmProcessorPrx::checkedCast(proxy);

			string request, response;
			/// \todo Add convenience function to create a request for given angles.
			/// Perhaps import paramset...
			boost::shared_ptr<const tmatrixParams> tmparams
				= tmatrixParams::create(tmatrixBase());
			queue::TM tm;
			queue::TMangles ang1(0,0,0,0);
			boost::shared_ptr<const vector<queue::TMangles> > 
				vang(new vector<queue::TMangles>(1,ang1));
			boost::shared_ptr<const queue::TMrequest> req1 = 
				queue::TMrequest::generate(tmparams, 0, 0, vang);
			tm.addTask(req1);

			Ryan_Serialization::writeString<queue::TM>(tm,request);

			cerr << "Submitting request\n";
			cerr << request << endl;
			Ice::AsyncResultPtr resptr = worker->begin_submit(request);
			cerr << "Request submitted\n";
			cerr << "Checking for result\n";
			response = worker->end_submit(resptr);
			cerr << response << endl;

			//Ice::ObjectPrx proxy = 
			//	communicator()->stringToProxy("IceGrid/Query");
			//IceGrid::QueryPrx query = IceGrid::QueryPrx::checkedCast(proxy);
		}
		catch (std::exception &e)
		{
			cerr << e.what() << endl;
			return 1;
		}
		return 0;
	}
};

int main(int argc, char** argv)
{
	TMclient app;
	return app.main(argc, argv, "config.client");
}

