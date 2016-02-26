///////////////////////////////////////////////////
// Low Level Parallel Programming 2016.
//
//     ==== Don't change this file! ====
//
// The main starting point for the crowd simulation.

#include <Windows.h>
#include "ped_model.h"
#include "MainWindow.h"
#include "ParseScenario.h"

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QApplication>
#include <QTimer>
#include <thread>

#include "PedSimulation.h"
#include <iostream>
#include <chrono>
#include <ctime>
#include <cstring>
#pragma comment(lib, "libpedsim.lib")
////
int main(int argc, char*argv[]) { 
	bool timing_mode = 0;
	QString scenefile = "scenario.xml";
	Ped::IMPLEMENTATION implementation = Ped::IMPLEMENTATION::SEQ;

	int i = 1;
	// Argument handling
	while (i < argc)
	{
		if (argv[i][0] == '-')
		{
			if (argv[i][1] == '-') {
				if (strcmp(&argv[i][2], "timing-mode") == 0)
				{
					cout << "Timing mode on\n";
					timing_mode = true;
				}
				else if (strcmp(&argv[i][2], "help") == 0)
				{
					cout << "Usage: " << argv[0] << " [--help] [--timing-mode] [scenario]" << endl;
					return 0;
				}
				else
				{
					cerr << "Unrecognized command: \"" << argv[i] << "\". Ignoring ..." << endl;
				}
			}
			else if (strcmp(&argv[i][1], "seq") == 0) {
				implementation = Ped::IMPLEMENTATION::SEQ;
			}
			else if (strcmp(&argv[i][1], "thr") == 0) {
				implementation = Ped::IMPLEMENTATION::PTHREAD;
			}
			else if (strcmp(&argv[i][1], "omp") == 0) {
				implementation = Ped::IMPLEMENTATION::OMP;
			}
			else if (strcmp(&argv[i][1], "vec") == 0) {
				implementation = Ped::IMPLEMENTATION::VECTOR;
			}
			else if (strcmp(&argv[i][1], "ocl") == 0) {
				implementation = Ped::IMPLEMENTATION::OCL;
			}
			else if (strcmp(&argv[i][1], "seq_col") == 0) {
				implementation = Ped::IMPLEMENTATION::SEQ_COL;
			}
			else if (strcmp(&argv[i][1], "omp_col") == 0) {
				implementation = Ped::IMPLEMENTATION::OMP_COL;
			}
		}
		else // Assume it is a path to scenefile
		{
			scenefile = argv[i];
		}

		i += 1;
	}

	cout << "Scenario: " << scenefile.toStdString() << endl;
	switch (implementation)
	{
	case Ped::OCL:
		cout << "Implementation: " << "OpenCL, no collision" << endl;
		break;
	case Ped::VECTOR:
		cout << "Implementation: " << "Vectorized, no collision" << endl;
		break;
	case Ped::OMP:
		cout << "Implementation: " << "OpenMP, no collision" << endl;
		break;
	case Ped::PTHREAD:
		cout << "Implementation: " << "Threads, no collision" << endl;
		break;
	case Ped::SEQ:
		cout << "Implementation: " << "Sequential, no collision" << endl;
		break;
	case Ped::SEQ_COL:
		cout << "Implementation: " << "Sequential, with collision" << endl;
		break;
	case Ped::OMP_COL:
		cout << "Implementation: " << "OpenMP, with collision" << endl;
		break;
	default:
		cout << "Implementation: " << "???" << endl;
		break;
	}

	// Reading the scenario file and setting up the crowd simulation model
	Ped::Model model;
	ParseScenario parser(scenefile);
	model.setup(parser.getAgents(), implementation);

	// GUI related set ups
	QApplication app(argc, argv);
	MainWindow mainwindow(model);

	// Default number of steps to simulate
	const int maxNumberOfStepsToSimulate = 5000;
	PedSimulation *simulation = new PedSimulation(model, mainwindow);

	cout << "Demo setup complete, running ..." << endl;

	int retval = 0;
	// Timing of simulation

	SYSTEMTIME start, stop;
	GetSystemTime(&start);

	if (timing_mode)
	{
		// Simulation mode to use when profiling (without any GUI)
		simulation->runSimulationWithoutQt(maxNumberOfStepsToSimulate);
	}
	else
	{
		// Simulation mode to use when visualizing
		mainwindow.show();
		simulation->runSimulationWithQt(maxNumberOfStepsToSimulate);
		retval = app.exec();
	}

	// End timing
	GetSystemTime(&stop);

	WORD duration = (stop.wMinute - start.wMinute) * 60000 + (stop.wSecond - start.wSecond) * 1000 + (stop.wMilliseconds - start.wMilliseconds);
	cout << "Time: " << duration <<  " milliseconds." << endl;
	cout << "Time (min:sec.millis): " << stop.wMinute - start.wMinute << ":" << stop.wSecond - start.wSecond << "." << stop.wMilliseconds - start.wMilliseconds << endl;
	cout << "Done" << endl;
	cout << "Type Enter to quit.." << endl;

	getchar(); // Wait for any key. Windows convenience...
	delete (simulation);
	return retval;
}
