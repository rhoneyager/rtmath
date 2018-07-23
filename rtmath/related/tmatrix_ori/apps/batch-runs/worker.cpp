#include <iostream>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include "worker.h"
#include "comm.h"
#include "dispatcher.h"
#include "debug.h"
#include "../../src/headers/tmatrix.h"
#include "../../src/headers/serialization.h"
#include "../../src/headers/tmatrix-serialization.h"

namespace tmatrix {
	Worker::Worker(QObject *parent) : QObject(parent)
	{
		_ppid = 0;
		_id = 0;
		_trd = 0;
		_wkr = 0;
		_isTerminating = false;
	}

	void Worker::attach()
	{
		using namespace std;
		// Check that _ppid is net. If not, die.
		if (!_ppid)
		{
			cerr << "Cannot attach without ppid setting!\n";
			exit(1);
		}
		cerr << "Attaching to ppid " << _ppid << " with id " << _id << endl;

		//std::unique_ptr<process> p(new process(_ppid, _id));
		std::unique_ptr<process> p(new process(_ppid, _id));
		_parent.swap(p);
		_parent->connect(true);

		attached();
	}

	void Worker::_wkrDone()
	{
		// Send a message back to the dispatcher, indicating that processing has
		// now completed.
		//message *nOut = new(_parent->_bOut->data()) message(DONE);
		std::string out;
		message msg(DONE);
		serialization::writeString<tmatrixOutVars>(_wkr->_out->res, msg._data);
		serialization::writeString<message>(msg,out);

		std::copy(out.data(),out.data()+out.size(), (char*) _parent->_bOut->data());

		_parent->_sOut->release();
	}

	void Worker::_mloop()
	{
		using namespace std;
		// Wait for a message from the parent.
		// Once message is recieved, process it.
		// Then, respond appropriately and reset the flag, causing the primary thread to block.
		// The parent does all of the queueing.
		// If more advanced processing is required, split a new thread and schedule a response.

		for (;;)
		{
			// acquire is a blocking call. It will interfere with timers, so it should be in
			// a separate thread.
			if(_parent->_sIn->acquire())
			{
				if (_isTerminating) 
				{
					std::cerr << "_mloop returning\n";
					return;
				}
				// Read in the message out of shared memory _bIn.
				std::string sin((char*) _parent->_bIn->data());
				message in;
				serialization::readString<message>(in,sin);
				//message *in = (message*) _parent->_bIn->data();

				switch (in.id())
				{
				case TERMINATE:
					std::cerr << "Closing on termination signal from master.\n";
					return;
					break;
				case START:
					// Have the main program begin job processing
					if (!_wkr)
					{
						istringstream i(string(in._data));
						boost::shared_ptr<tmatrixInVars> m;
						boost::shared_ptr<tmatrixAngleRes> n;
						serialization::read<boost::shared_ptr<tmatrixInVars> >(m,i);
						serialization::read<boost::shared_ptr<tmatrixAngleRes> >(n,i);

						_wkr = new WorkerThread(this, m, n);
						QObject::connect(_wkr, SIGNAL( finished() ), this,
							SLOT( _wkrDone() ), Qt::QueuedConnection);
					}
					_wkr->start();
					// Once job processing is done, send DONE signal
					// and then switch to sending NOOP
					break;
				case NOOP:
				default:
					break;
				}

			} else {
				// Semaphore could not be acquired. Die.
				std::cerr << "Semaphore lost\n";
				exit(1);
			}
		}
	}

	void Worker::close()
	{
		_isTerminating = true;
		if (_trd)
		{
			if (_trd->isRunning())
			{
				_parent->_sIn->release(5);
				// Block until thread stops for 0.5 seconds
				// Then terminate the thread
				if (_trd->wait(500) == false)
				{
					std::cerr << "Stuck in _mloop. Forcibly terminating.\n";
					_trd->terminate();
				}
			}
		}
		terminated();
		exit(0);
	}

	void Worker::setParent(unsigned int pid, unsigned int myid)
	{
		_ppid = pid;
		_id = myid;
	}

	void Worker::load()
	{
		std::cerr << "Loading\n";
		// Set a timer to verify that the parent still exists. If not, then terminate the
		// worker. 
		QObject::connect(&_lifeTimer, SIGNAL( timeout() ), this, 
			SLOT( verifyParent() ), Qt::DirectConnection);
		_lifeTimer.start(20000);


		// attach will invoke startLoop, which starts the blocking message parser on a separate thread.
		QTimer::singleShot(10000, this, SLOT(attach() ) );
	}

	void Worker::verifyParent()
	{
		if (!debug::pidExists(_ppid))
		{
			std::cerr << "Parent cannot be verified. Terminating.\n";
			close();
		} else {
			//std::cerr << "Parent exists!" << std::endl;
		}
	}

	void Worker::startLoop()
	{
		if (!_trd)
			_trd = new WorkerProcessor(this);
		// Setup signal that _trd stopping kills main process as well...
		QObject::connect(_trd, SIGNAL( finished() ), this, 
			SLOT( close() ), Qt::QueuedConnection);
		_trd->start();
	}

	void WorkerProcessor::run()
	{
		_wkr->_mloop();
	}

	void WorkerThread::run()
	{
		tmatrix tm(*_in);
		tm.run();
		_out->res = tm.outs;
	}


}

