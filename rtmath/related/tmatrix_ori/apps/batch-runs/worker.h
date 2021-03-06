#pragma once
/* The main header for the qt implementation
* of the worker process.*/

#include <QtCore/QObject>
#include <QtCore/QSharedMemory>
#include <QtCore/QSystemSemaphore>
#include <QtCore/QThread>
#include <QtCore/QTimer>
#include <memory>
#include "comm.h"

namespace tmatrix {

	class WorkerProcessor;
	class WorkerThread;

	class Worker : public QObject
	{
		Q_OBJECT
	public:
		explicit Worker(QObject *parent = 0);
signals:
		void terminated();
		void loaded();
		void attached();
		public slots:
			void attach();
			void close();
			void load();
			void startLoop();
			void verifyParent();
			private slots:
				void _mloop();
				void _wkrDone();
	public:
		void setParent(unsigned int pid, unsigned int myid);
	private:
		unsigned int _ppid;
		unsigned int _id;
		bool _isTerminating;
		std::unique_ptr<process> _parent;
		WorkerProcessor *_trd;
		WorkerThread *_wkr;
		QTimer _lifeTimer;
		friend class WorkerProcessor;
		friend class WorkerThread;
	};

	// Spawns the worker message loop in a separate thread 
	// (for parent communication without blocking)
	class WorkerProcessor : public QThread
	{
	public:
		WorkerProcessor(Worker* wkr) : _wkr(wkr) {}
		void run();
	private:
		Worker *_wkr;
	};

	// The threaded worker part that calls tmatrix
	class WorkerThread : public QThread
	{
	public:
		WorkerThread(Worker* wkr, boost::shared_ptr<tmatrixInVars> &m, 
			boost::shared_ptr<tmatrixAngleRes> &n) 
			: _wkr(wkr), _in(m), _out(n) {}
		void run();
		boost::shared_ptr<tmatrixInVars> _in;
		boost::shared_ptr<tmatrixAngleRes> _out;
	private:
		Worker *_wkr;
	};

}
