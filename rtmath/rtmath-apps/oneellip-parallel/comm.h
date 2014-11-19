#pragma once

#include <array>
#include <QObject>
#include <QSharedMemory>
#include <QSystemSemaphore>
#include <QThread>
#include <QTimer>
#include <QDebug>
#include <string>
#include <memory>

#include <Ice/Ice.h>
#include <tmm.h>

namespace rtmath {
	namespace apps {
		namespace oneellipParallel {
#define oneellip_parallel_maxMessageSize 32768
			enum class ptype_t {
				DISPATCHER,
				WORKER
			};

			Ice::CommunicatorPtr defaultCommunicator;

			class process;

			class processBase : public std::enable_shared_from_this < processBase >
			{
			public:
				processBase();
				virtual ~processBase();
				ptype_t type() const;
				int pid() const;
				int suffix() const;
				/// Initiales the connection, as either the shared 
				/// memory provider or client.
				void connect(bool swap = false);
				bool connected() const;
				void kill();
				void exec(const message &msg);
			public:
				int _pid;
				int _suffix;
				bool _connected;
				ptype_t _type;
				std::string _inKey, _outKey;
				std::shared_ptr<process> _parent;
				/// \note Not using the QSharedMemory semaphores because they are fully-blocking
				/// (no try method) and they lack the ability to have multiple locks.
				std::shared_ptr<QSystemSemaphore> _sIn, _sOut;
				/// The input / output stream shared memory locations contain class instances of 
				/// messages (described below)
				std::shared_ptr<QSharedMemory> _bIn, _bOut;
			};

			class process : public processBase
			{
			public:
				process();
				process(int pid, int suffix, ptype_t type = ptype_t::DISPATCHER);
				virtual ~process();
			};


			void testCreate(std::shared_ptr<QSharedMemory> *tgt, size_t size, const char* name);

		}
	}
}

