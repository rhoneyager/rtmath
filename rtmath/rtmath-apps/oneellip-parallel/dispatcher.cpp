#include <iostream>
#include <QApplication>
#include <QProcess>
#include <QObject>
#include <QDataStream>
#include <QFile>
#include <sstream>
#include <boost/filesystem.hpp>
#include <Ryan_Debug/debug.h>
#include "dispatcher.h"

namespace rtmath {
	namespace apps {
		namespace oneellipParallel {

			Dispatcher::Dispatcher(const std::string &appname,
				size_t numcpus, QObject *parent) : QObject(parent)
			{
				_processesLoaded = false;
				_numWorkers = 0;
				_numActiveWorkers = 0;
				_numCPUS = 0;
				_modelRunning = false;
				_pid = QCoreApplication::applicationPid();
				sQueue.release(1);

				if (parent)
					QObject::connect(this, SIGNAL(terminated()), parent,
					SLOT(quit()), Qt::QueuedConnection);
				QObject::connect(this, SIGNAL(loaded()), this,
					SLOT(startModel()), Qt::QueuedConnection);
				//QObject::connect(this, SIGNAL(modelDone()), this,
				//	SLOT(close()), Qt::QueuedConnection);
				QObject::connect(this, SIGNAL(subprocessDone(int)), this,
					SLOT(schedule(int)), Qt::QueuedConnection);
				QTimer::singleShot(0, this, SLOT(initialize()));
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

				for (size_t i = 0; i < num; i++)
				{
					int pid = Ryan_Debug::getPID();
					boost::shared_ptr<process> np(new process(pid, (int)_numWorkers, ptype_t::WORKER));
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

					QObject::connect(c, SIGNAL(finished(int)), this, SLOT(workerTerminated(int)));
					//std::cout << qPrintable(sArgs) << std::endl;
					c->start(
						_appname.c_str(),
						args
						);

					_processesExec.insert(boost::shared_ptr<QProcess>(c));

					// Initialize the shared memory region
					// (give it a size and construct a default object)
					_processes[_numWorkers] = np;
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
				//serialization::write< vector<tmatrixSet> >(_tmatrices, _ofile);
			}

			void Dispatcher::load()
			{
				/*
				using namespace std;
				vector <tmatrixSet> loader;
				for (auto it = _files.begin(); it != _files.end(); ++it)
				{
				serialization::read< vector<tmatrixSet> >(loader, *it);
				_tmatrices.resize(_tmatrices.size() + loader.size());
				std::copy(loader.begin(), loader.end(), _tmatrices.rbegin());
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

				_queue.push(std::pair < boost::shared_ptr<tmatrixInVars>,
				boost::shared_ptr<tmatrixAngleRes> >
				(ineff, *ot));
				}
				}
				*/
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
				for (size_t i = 0; i < _numWorkers; i++)
					schedule((int)i);
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

				DispatcherProcessor *dp = new DispatcherProcessor(id, _processes[id], this, ntm);
				_processesHandler[id] = boost::shared_ptr<DispatcherProcessor>(dp);

				dp->start(); // exec the command and await for response from child process on a separate thread...

				sQueue.release();
			}

			void Dispatcher::setAppName(const std::string &fname)
			{
				_appname = fname;
			}

			void DispatcherProcessor::run()
			{
				using namespace std;
				_in->id = START;
				_p->exec(_in);

				for (;;)
				{
					if (_p->_sIn->acquire())
					{
						// Thread blocks until the response is received

						std::vector<Ice::Byte> data;
						std::copy_n((char*)_p->_bIn->data(),
							oneellip_parallel_maxMessageSize,
							data.end());

						auto ins = Ice::createInputStream(defaultCommunicator, data);
						ins->read(*_in);

						switch (_in->id)
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
							_src->_results.push_back(_in);
							_src->emitSubprocessDone((int)_id);
							return;
							break;
						default:
							// Unknown signal received
							std::cerr << "child " << _id << " has sent an abnormal response.\n";
							std::cerr << "\tCode: " << _in->id << std::endl;
							break;
						}

					}
					else {
						// Semaphore could not be acquired. Die.
						std::cerr << "Semaphore lost on id " << _id << "\n";
						exit(1);
					}
				}
			}

			int Dispatcher::numProcessors(const ::Ice::Current&) const
			{
				return _numCPUS;
			}

			std::vector<message> Dispatcher::doRun(const inputs& input,
				const ::Ice::Current& c) const
			{
				std::vector<message> res;

				for (const auto &i : input)
				{
					std::shared_ptr<message> m(new message);
					m->in = i;
					m->id = START;
					_queue.push(m);
				}

				startModel();

				// Take model outputs and return to caller
				// Results are in _results

				return res;
			}


		}
	}
}
