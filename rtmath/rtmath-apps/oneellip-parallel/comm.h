#pragma once

#include <QObject>
#include <QSharedMemory>
#include <QSystemSemaphore>
#include <QThread>
#include <QTimer>
#include <QDebug>
#include <string>
#include <memory>

namespace tmatrix {

	enum ptype_t {
		DISPATCHER,
		WORKER,
		NUM_PTYPES};

		// boost can index these!

		class process;
		class message;

		class processBase : public std::enable_shared_from_this<processBase>
		{
		public:
			processBase();
			virtual ~processBase();
			ptype_t type() const;
			int pid() const;
			int suffix() const;
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
			// Not using the QSharedMemory semaphores because they are fully-blocking
			// (no try method) and they lack the ability to have multiple locks.
			std::shared_ptr<QSystemSemaphore> _sIn, _sOut;
			// The input / output stream shared memory locations contain class instances of 
			// messages (described below)
			std::shared_ptr<QSharedMemory> _bIn, _bOut;
		};

		class process : public processBase
		{
		public:
			process();
			process(int pid, int suffix, ptype_t type = DISPATCHER);
			virtual ~process();
		};

		enum messageId
		{
			NOOP,
			START,
			DONE,
			TERMINATE
		};

		class message
		{
		public:
			message(messageId nid = NOOP)
				:
			_id(nid)
			{}
			messageId id() const { return _id; }
			//private:
			messageId _id;
			std::string _data;
		};

		void testCreate(std::shared_ptr<QSharedMemory> *tgt, size_t size, const char* name);

}

