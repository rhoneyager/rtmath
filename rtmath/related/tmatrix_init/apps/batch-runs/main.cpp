#include <QtCore/QObject>
#include <QtCore/QTimer>
#include <QtGui/QApplication>
#include <QtCore/QProcess>
#include <iostream>
// Using boost parameter processing this time to avoid rtmath dependencies
#include <boost/program_options.hpp>
#include <vector>
#include <string>
#include "debug.h"
#include "dispatcher.h"
#include "worker.h"

int main(int argc, char *argv[])
{
	using namespace std;
	
	try {
		QCoreApplication a(argc, argv);

		string appexec(argv[0]);

		// Do processing of argv
		namespace po = boost::program_options;
		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("ppid,p", po::value<int>(), "set invoker parent id")
			("num-cpus,c", po::value<size_t>(), "set number of workers")
			("input-file,i", po::value< vector<string> >(), "input file")
			("output-file,o", po::value<string>(), "Set output file (required)")
		;
		po::positional_options_description p;
		p.add("input-file", -1);

		po::variables_map vm;
		po::store(po::command_line_parser(argc,argv).
			options(desc).positional(p).run()
			, vm);
		po::notify(vm);

		// Begin checking parameters
		if (vm.count("help") || vm.size() == 0)
		{
			cout << desc << endl;
			exit(3);
		}

		int ppid = 0;
		int myid = 0;
		int numcpus = 0;
		if (vm.count("ppid"))
		{
			myid = vm["ppid"].as<int>();
			ppid = tmatrix::debug::getPPID();
			int pid = tmatrix::debug::getPID();
			cout << "Subprocess " << myid << " has pid " << pid << " with ppid " << ppid << endl;
		}

		if (vm.count("num-cpus"))
		{
			numcpus = vm["num-cpus"].as<size_t>();
		} else {
			numcpus = 1;
		}

		string ofile;
		vector<string> files;
		if (vm.count("input-file"))
		{
			files = vm["input-file"].as< vector<string> >();
		}

		if (vm.count("output-file"))
		ofile = vm["output-file"].as<string>();

		// If no ppid, then this is the parent process.
		// Spawn message loop, create children, and process files.
		// If ppid, then this is a child. Connect to parent.
		if (ppid)
		{
			tmatrix::Worker worker;
			worker.setParent(ppid, myid);
			QObject::connect(&worker, SIGNAL( terminated() ), &a, 
					SLOT( quit() ), Qt::QueuedConnection);
			QObject::connect(&worker, SIGNAL( attached() ), &worker, 
					SLOT( startLoop() ), Qt::QueuedConnection);
			QObject::connect(&a, SIGNAL( aboutToQuit() ), &worker, 
					SLOT( close() ), Qt::DirectConnection);
			QTimer::singleShot(0, &worker, SLOT(load() ) );
			return a.exec();
		} else {
			cerr << "tmatrix-batch-runs" << endl;
			tmatrix::debug::appEntry(argc,argv);
			tmatrix::debug::debug_preamble();
			cout << "Running with " << numcpus << " workers." << endl;

			cout << "Input files are:";
			for (auto it = files.begin(); it != files.end(); it++)
				cout << " " << *it;
			cout << endl;

			cout << "Outputting to: " << ofile << endl;

			tmatrix::Dispatcher dispatcher;

			dispatcher.setAppName(appexec);
			dispatcher.setFiles(files,ofile);
			dispatcher.setNumCPUS(numcpus);

			QObject::connect(&dispatcher, SIGNAL( terminated() ), &a, 
				SLOT( quit() ), Qt::QueuedConnection);
			QObject::connect(&dispatcher, SIGNAL( loaded() ), &dispatcher, 
				SLOT( startModel() ), Qt::QueuedConnection);
			QObject::connect(&dispatcher, SIGNAL( modelDone() ), &dispatcher, 
				SLOT( close() ), Qt::QueuedConnection);
			QObject::connect(&dispatcher, SIGNAL( subprocessDone(int) ), &dispatcher,
				SLOT( schedule(int) ), Qt::QueuedConnection);
			QTimer::singleShot(0, &dispatcher, SLOT( initialize() ) );

			return a.exec();
		}

	} 
	catch (std::exception &e)
	{
		cerr << "Exception: " << e.what() << endl;
		exit(2);
	}
	catch (...)
	{
		cerr << "Caught unidentified error... Terminating." << endl;
		exit(1);
	}
}


