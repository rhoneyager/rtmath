#include <iostream>
#include <sstream>
#include <Ryan_Debug/debug.h>
#include "../../rtmath/rtmath/phaseFunc.h"
#include "worker.h"
#include "comm.h"
#include "dispatcher.h"

namespace rtmath {
	namespace apps {
		namespace oneellipParallel {
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

				_wkr->_in->id = messageId::DONE;
				std::vector<Ice::Byte> data;
				auto out = Ice::createOutputStream(defaultCommunicator);
				out->write(*(_wkr->_in));
				out->finished(data);
				std::copy_n(data.data(), data.size(),
					(char*)_parent->_bOut->data());

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
					if (_parent->_sIn->acquire())
					{
						if (_isTerminating)
						{
							std::cerr << "_mloop returning\n";
							return;
						}
						// Read in the message out of shared memory _bIn.
						//std::string sin((char*)_parent->_bIn->data());
						std::shared_ptr<message> in(new message);

						std::vector<Ice::Byte> data;
						std::copy_n((char*)_parent->_bIn->data(),
							oneellip_parallel_maxMessageSize,
							data.end());
						
						auto ins = Ice::createInputStream(defaultCommunicator, data);
						ins->read(*in);

						switch (in->id)
						{
						case TERMINATE:
							std::cerr << "Closing on termination signal from master.\n";
							return;
							break;
						case START:
							// Have the main program begin job processing
							if (!_wkr)
							{
								_wkr = new WorkerThread(this, in);
								QObject::connect(_wkr, SIGNAL(finished()), this,
									SLOT(_wkrDone()), Qt::QueuedConnection);
							}
							_wkr->start();
							// Once job processing is done, send DONE signal
							// and then switch to sending NOOP
							break;
						case NOOP:
						default:
							break;
						}

					}
					else {
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
				QObject::connect(&_lifeTimer, SIGNAL(timeout()), this,
					SLOT(verifyParent()), Qt::DirectConnection);
				_lifeTimer.start(20000);


				// attach will invoke startLoop, which starts the blocking message parser on a separate thread.
				QTimer::singleShot(10000, this, SLOT(attach()));
			}

			void Worker::verifyParent()
			{
				if (!Ryan_Debug::pidExists(_ppid))
				{
					std::cerr << "Parent cannot be verified. Terminating.\n";
					close();
				}
				else {
					//std::cerr << "Parent exists!" << std::endl;
				}
			}

			void Worker::startLoop()
			{
				if (!_trd)
					_trd = new WorkerProcessor(this);
				// Setup signal that _trd stopping kills main process as well...
				QObject::connect(_trd, SIGNAL(finished()), this,
					SLOT(close()), Qt::QueuedConnection);
				_trd->start();
			}

			void WorkerProcessor::run()
			{
				_wkr->_mloop();
			}

			void WorkerThread::run()
			{
				using namespace rtmath::phaseFuncs;
				pf_class_registry::orientation_type o = pf_class_registry::orientation_type::ISOTROPIC;
				pf_class_registry::inputParamsPartial i;
				i.aeff = _in->in.aeff;
				i.aeff_rescale = _in->in.aeffRescale;
				// TODO: cast shape version
				//i.aeff_version = _in->in.aeffVersion;
				i.aeff_version = pf_class_registry::inputParamsPartial::aeff_version_type::EQUIV_V_SPHERE;
				i.eps = _in->in.eps;
				i.m = std::complex<double>(_in->in.mRe, _in->in.mIm);
				i.ref = _in->in.ref;
				// Figure out function to use from refractive index specification
				// TODO!
				i.rmeth = rmeth; // Yeah, only one refractive index method per program invocation is supported.
				// TODO: cast shape type
				i.shape = pf_class_registry::inputParamsPartial::shape_type::SPHEROID;

				i.vFrac = _in->in.vFrac;


				pf_provider p(o, i);

				pf_provider::resCtype res;
				pf_class_registry::setup s;
				s.beta = 0; s.theta = 0; s.phi = 0;
				s.sPhi = 0; s.sPhi0 = 0; s.sTheta = 0; s.sTheta0 = 0;
				s.wavelength = _in->in.lambda;

				pf_provider::resCtype csout;
				p.getCrossSections(s, csout);
				for (const auto &c : csout)
				{
					crossSections o;
					o.valid = c.second.valid;
					o.provider = std::string(c.first);
					o.g = c.second.g_iso;
					o.Qabs = c.second.Qabs_iso;
					o.Qext = c.second.Qext_iso;
					o.Qsca = c.second.Qsca_iso;
					o.Qbk = c.second.Qbk_iso;
					_in->cs.push_back(o);
				}
			}


		}
	}
}
