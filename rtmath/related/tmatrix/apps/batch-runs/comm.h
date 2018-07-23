#pragma once

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/complex.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/version.hpp>

#include <QtCore/QObject>
#include <QtCore/QSharedMemory>
#include <QtCore/QSystemSemaphore>
#include <QtCore/QThread>
#include <QtCore/QTimer>
#include <QtCore/QDebug>
#include <string>
#include <memory>
#include "../../src/headers/tmatrix.h"

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
		private:
			friend class boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version)
			{
				ar & boost::serialization::make_nvp("id", _id);
				ar & boost::serialization::make_nvp("data", _data);
			}
		};

		void testCreate(std::shared_ptr<QSharedMemory> *tgt, size_t size, const char* name);

}

