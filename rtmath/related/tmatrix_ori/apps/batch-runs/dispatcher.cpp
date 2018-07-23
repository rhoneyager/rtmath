#include <iostream>
#include <QtGui/QApplication>
#include <QtCore/QProcess>
#include <QtCore/QObject>
#include <QtCore/QDataStream>
//#include <qfile.h>
#include <QtCore/QFile>
#include <sstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/filesystem.hpp>
#include <boost/serialization/vector.hpp>
#include "dispatcher.h"
#include "../../src/headers/tmatrix.h"
#include "../../src/headers/tmatrix-serialization.h"
#include "../../src/headers/serialization.h"
#include "debug.h"

namespace tmatrix {

	Dispatcher::Dispatcher(QObject *parent) : QObject(parent)
	{
		_processesLoaded = false;
		_numWorkers = 0;
		_numActiveWorkers = 0;
		_numCPUS = 0;
		_modelRunning = false;
		_pid = QCoreApplication::applicationPid();
		sQueue.release(1);
	}

	Dispatcher::~Dispatcher()
	{
		// Kill any wayward children, just in case
		for (auto it = _processesExec.begin(); it != _processesExec.end(); it++)
		{
			if (!(*it)->waitForFinished(500))
				(*it)->kill();
		}
	}

	void Dispatcher::initialize()
	{
		// Add the worker processes
		addWorkerProcesses(_numCPUS);
		// Emit loaded signal
		loaded();
	}

	void Dispatcher::close()
	{
		// End all child processes
		stopProcesses();

		// Write out results
		write();

		// Emit signal to end program
		terminated();
	}

	void Dispatcher::workerTerminated(int err)
	{
		std::cerr << "Worker terminated with error code " << err << std::endl;
	}

	void Dispatcher::addWorkerProcesses(size_t num)
	{

		for (size_t i=0; i< num; i++)
		{
			int pid = debug::getPID();
			boost::shared_ptr<process> np(new process(pid, _numWorkers, WORKER));
			np->connect(false);


			QProcess *c = new QProcess();
			QStringList args;
			c->setProcessChannelMode(QProcess::ForwardedChannels); // Send streams to main app
			args << "-p";
			//std::cout << _numWorkers << std::endl;
			std::ostringstream p;
			p << _numWorkers;
			std::string sp = p.str();
			//std::cout << sp << std::endl;
			args << sp.c_str();

			QString sArgs = args.join(" ");

			QObject::connect(c, SIGNAL(finished(int)), this, SLOT( workerTerminated(int) ) );
			//std::cout << qPrintable(sArgs) << std::endl;
			c->start(
				_appname.c_str(), 
				args
				);

			_processesExec.insert( boost::shared_ptr<QProcess>(c));

			// Initialize the shared memory region
			// (give it a size and construct a default object)
			_processes[_numWorkers]=np;
			_numWorkers++;
		}
	}

	size_t Dispatcher::numWorkerProcesses() const
	{
		return _numWorkers;
	}

	void Dispatcher::stopProcesses()
	{
		// Send all workers the termination signal
		for (auto it = _processes.begin(); it != _processes.end(); it++)
		{
			(*it).second->kill();
		}
	}

	bool Dispatcher::modelRunning() const
	{
		return _modelRunning;
	}

	void Dispatcher::write() const
	{
		using namespace std;
		serialization::write< vector<tmatrixSet> >(_tmatrices, _ofile);
	}

	void Dispatcher::load()
	{
		using namespace std;
		vector <tmatrixSet> loader;
		for (auto it = _files.begin(); it != _files.end(); ++it)
		{
			serialization::read< vector<tmatrixSet> >(loader, *it);
			_tmatrices.resize(_tmatrices.size() + loader.size());
			std::copy(loader.begin(),loader.end(), _tmatrices.rbegin());
		}

		// Now that all tmatrices are loaded, insert the operations 
		// into the queue
		for (auto it = _tmatrices.begin(); it != _tmatrices.end(); ++it)
		{
			for (auto ot = it->results.begin(); ot != it->results.end(); ++ot)
			{
				boost::shared_ptr<tmatrixInVars> ineff(new tmatrixInVars(*(it->base)));
				ineff->ALPHA = (*ot)->alpha;
				ineff->BETA = (*ot)->beta;
				ineff->THET = (*ot)->theta;
				ineff->THET0 = (*ot)->theta0;
				ineff->PHI = (*ot)->phi;
				ineff->PHI0 = (*ot)->phi0;

				_queue.push(std::pair< boost::shared_ptr<tmatrixInVars> , 
					boost::shared_ptr<tmatrixAngleRes> >
					(ineff, *ot));
			}
		}
	}

	void Dispatcher::startModel()
	{
		if (!_numWorkers)
		{
			modelDone();
			return;
		}

		// Begin by loading the files. Parse files and 
		// populate the vector of tmatrices
		// Use boost and the tmatrix library for this...
		sQueue.acquire();

		load();

		sQueue.release();

		// Once all entries are loaded, schedule each process
		_numActiveWorkers = _numWorkers;
		for (size_t i=0; i < _numWorkers; i++)
			schedule(i);
	}

	void Dispatcher::schedule(int id)
	{
		// Lock the queue
		sQueue.acquire();

		if (!_queue.size())
		{
			_numActiveWorkers--;
			// Set the subprocess as inactive
			_processes[id]->kill();

			std::cerr << "Id " << id << " has no jobs left. Terminated." << std::endl;

			// If out of active workers, then the model is done
			if (!_numActiveWorkers)
				modelDone();

			sQueue.release();
			return;
		}
		// Pull from the vector _tmatrices
		auto ntm = _queue.front();
		_queue.pop();

		// Talk to subprocess, providing new tmatrix to process

		DispatcherProcessor *dp = new DispatcherProcessor(id, _processes[id], this, ntm.first, ntm.second);
		_processesHandler[id] = boost::shared_ptr<DispatcherProcessor>(dp);

		dp->start(); // exec the command and await for response from child process on a separate thread...

		sQueue.release();
	}

	void Dispatcher::setFiles(const std::vector<std::string> &files, const std::string &ofile)
	{
		_files = files;
		_ofile = ofile;
	}

	void Dispatcher::setAppName(const std::string &fname)
	{
		_appname = fname;
	}

	void DispatcherProcessor::run()
	{
		using namespace std;
		message nm(START);
		
		ostringstream body;

		serialization::write<boost::shared_ptr<tmatrixInVars> >(_in,body);
		serialization::write<boost::shared_ptr<tmatrixAngleRes> >(_out,body);
		nm._data = body.str();

		_p->exec(nm);

		for (;;)
		{
			if(_p->_sIn->acquire())
			{
				// Thread blocks until the response is received
				std::string sin((char*) _p->_bIn->data());
				message in;
				serialization::readString<message>(in,sin);
				//message *in = (message*) _p->_bIn->data();

				switch (in.id())
				{
				case TERMINATE:
					std::cerr << "Child id " << _id << " is terminating.\n";
					return;
					break;
				case NOOP:
					break;
				case DONE:
					// Child process has completed its task and is now done.
					// Save data!
					serialization::readString<tmatrixOutVars>(_out->res, in._data);
					_src->emitSubprocessDone(_id);
					return;
					break;
				default:
					// Unknown signal received
					std::cerr << "child " << _id << " has sent an abnormal response.\n";
					std::cerr << "\tCode: " << in.id() << std::endl;
					break;
				}

			} else {
				// Semaphore could not be acquired. Die.
				std::cerr << "Semaphore lost on id " << _id << "\n";
				exit(1);
			}
		}
	}


}

