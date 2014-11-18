#pragma once
/* This is the class that contains the 
* execution logic of the program. The 
* dispatcher loops are the first things 
* to be executed in the message loop.
*/

#include <QObject>
#include <QSharedMemory>
#include <QSystemSemaphore>
#include <QSemaphore>
#include <QProcess>
#include <set>
#include <vector>
#include <map>
#include <queue>
#include <boost/shared_ptr.hpp>
#include "comm.h"

namespace tmatrix
{

	class DispatcherProcessor;

	class Dispatcher : public QObject
	{
		Q_OBJECT
	public:
		explicit Dispatcher(QObject *parent = 0);
		virtual ~Dispatcher();
signals:
		void terminated();
		// Real signals
		void modelDone();
		void loaded();
		void subprocessDone(int id);
		public slots:
			// Real slots
			void addWorkerProcesses(size_t num = 1);
			void workerTerminated(int errCode);
			void startModel();
			void initialize();
			void close();
			void schedule(int id);
	private:
		void stopProcesses();
		void load();
		bool _processesLoaded;
		size_t _numWorkers;
		size_t _numActiveWorkers;
		bool _modelRunning;
		int _pid;
		size_t _numCPUS;
		std::map<size_t, boost::shared_ptr<process> > _processes;
		std::map<size_t, boost::shared_ptr<DispatcherProcessor> > _processesHandler;
		//std::map<size_t, tmatrix::tmatrixOutVars> _results;
		std::set<boost::shared_ptr<QProcess> > _processesExec;
		//std::map<size_t, std::shared_ptr<QTimer> > _pLifeTimer;
		std::vector<std::string> _files; // Input files
		std::string _ofile; // Output file
		std::string _appname;

		std::vector< tmatrixSet > _tmatrices;
		std::queue< std::pair< boost::shared_ptr<tmatrixInVars> , 
			boost::shared_ptr<tmatrixAngleRes> > > _queue;
		//std::queue< std::shared_ptr<tmatrix::tmatrix> > _tmatrices;
		QSemaphore sQueue;
	public:
		size_t numWorkerProcesses() const;
		bool modelRunning() const;
		void write() const;
		void setFiles(const std::vector<std::string> &files, const std::string &ofile);
		void setAppName(const std::string &fname);
		void setNumCPUS(size_t numcpus) { _numCPUS = numcpus; }
		void emitSubprocessDone(int id) { subprocessDone(id); }
		// Model will be stored here
		// Fortran-specific stuff goes in a worker
		// Hint: this necessitates that shared memory
		// be handled by the dispatcher
		friend class DispatcherProcessor;
	};

	class DispatcherProcessor : public QThread
	{
	public:
		DispatcherProcessor(size_t id, 
			boost::shared_ptr<process> p, 
			Dispatcher* src, 
			boost::shared_ptr<tmatrixInVars> &in,
			boost::shared_ptr<tmatrixAngleRes> &out) 
			: _p(p), _id(id), _src(src), _in(in), _out(out) {}
		void run();
	private:
		boost::shared_ptr<process> _p;
		size_t _id;
		boost::shared_ptr<tmatrixInVars> _in;
		boost::shared_ptr<tmatrixAngleRes> _out;
		Dispatcher *_src;
	};

}

