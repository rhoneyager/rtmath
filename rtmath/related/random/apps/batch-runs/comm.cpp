#include <sstream>
#include <iostream>
#include <algorithm>
#include "comm.h"
#include "../../src/headers/serialization.h"

namespace tmatrix {

	const size_t maxMessageSize = 32768;

	processBase::processBase()
	{
		_pid = 0;
		_type = WORKER;
		_sIn = 0;
		_sOut = 0;
		_bIn = 0;
		_bOut = 0;
		_suffix = 0;
		_connected = false;
	}

	processBase::~processBase()
	{
	}

	int processBase::pid() const
	{
		return _pid;
	}

	int processBase::suffix() const
	{
		return _suffix;
	}

	bool processBase::connected() const
	{
		return _connected;
	}

	ptype_t processBase::type() const
	{
		return _type;
	}

	void testCreate(std::shared_ptr<QSharedMemory> *tgt, size_t size, const char* name)
	{
		bool flag = false;
		flag = (*tgt)->create(size);
		if (!flag)
		{
			QSharedMemory::SharedMemoryError err;
			err = (*tgt)->error();
			if (err == QSharedMemory::AlreadyExists)
			{
				if (!(*tgt)->attach())
				{
					qDebug() << "QSharedMemory " << name << " already exists and cannot attach.\n";
					throw;
				}
			} else {
				qDebug() << "Creation of " << name << " failed with following error:\n";
				qDebug() << (*tgt)->errorString();
				throw;
			}
		}
	}

	void processBase::connect(bool swap)
	{
		using namespace std;
		ostringstream s1name, s2name, b1name, b2name, bfname, bvname;
		s1name << "s-" << _pid << "-" << _suffix << "-1";
		s2name << "s-" << _pid << "-" << _suffix << "-2";
		b1name << "b-" << _pid << "-" << _suffix << "-1";
		b2name << "b-" << _pid << "-" << _suffix << "-2";
		string name1 = s1name.str();
		string name2 = s2name.str();
		string bname1 = b1name.str();
		string bname2 = b2name.str();

		// Semaphore access flags depend on swap
		QSystemSemaphore::AccessMode am;
		if (!swap)
		{
			am = QSystemSemaphore::Create;
		} else {
			am = QSystemSemaphore::Open;
		}

		_sIn = std::shared_ptr<QSystemSemaphore>(new QSystemSemaphore(name1.c_str(),0,am));
		_sOut = std::shared_ptr<QSystemSemaphore>(new QSystemSemaphore(name2.c_str(),0,am));

		_bIn = std::shared_ptr<QSharedMemory>(new QSharedMemory(bname1.c_str()));
		_bOut = std::shared_ptr<QSharedMemory>(new QSharedMemory(bname2.c_str()));

		if (swap)
		{
			_sIn.swap(_sOut);
			_bIn.swap(_bOut);
		}

		// Swap determines who creates and who attaches to the shared memory segment
		if (!swap)
		{
			//cout << sizeof(message) << endl;
			// void testCreate(std::shared_ptr<QSharedMemory> *tgt, size_t size, const char* name)

			testCreate(&_bIn, maxMessageSize, "_bIn");
			testCreate(&_bOut, maxMessageSize, "_bOut");

			// And create a default message
			// First time I've used placement new!
			message *nIn = new(_bIn->data()) message;
			message *nOut = new(_bOut->data()) message;

		} else {
			if (!_bIn->attach()) throw;
			if (!_bOut->attach()) throw;
		}

		_connected = true;
	}

	void processBase::exec(const message & msg)
	{
		// Serialize the message and then release
		//message *nm = new(_processes[id]->_bOut->data()) message(START);
		std::string out;
		serialization::writeString<message>(msg,out);
		std::copy(out.data(), out.data()+out.size(), (char*) _bOut->data());
		//  _bOut->data()

		_sOut->release();
	}

	void processBase::kill()
	{
		std::string out;
		serialization::writeString<message>(message(TERMINATE),out);
		std::copy(out.data(), out.data()+out.size(), (char*) _bOut->data());
		//message *nIn = new(_bOut->data()) message(TERMINATE);
		_sOut->release();
	}

	process::process()
	{
	}

	process::process(int pid, int suffix, ptype_t type)
	{
		_pid = pid;
		_suffix = suffix;
		_type = type;
	}

	process::~process()
	{
	}


}
