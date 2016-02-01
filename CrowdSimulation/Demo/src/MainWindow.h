#ifndef _mainwindow_h_
#define _mainwindow_h_

#include <QMainWindow>
#include <QGraphicsScene>

#include <vector>

#include "ped_model.h"
#include "ped_agent.h"
#include "ViewAgent.h"
class QGraphicsView;

////////////
/// Comment in for Assignment 4
///////////////////////////////////////////////
// #define ASSIGNMENT_4

class MainWindow : public QMainWindow {
private:
	MainWindow();
public:
	
	MainWindow(const Ped::Model &model);

	// paint is called after each computational step
	// to repaint the window
	void paint();

	static int cellToPixel(int val);
	static const int cellsizePixel = 5;
private:
	QGraphicsView *graphicsView;
	QGraphicsScene * scene;

	const Ped::Model &model;

	// the graphical representation of each agent
	std::vector<ViewAgent*> viewAgents;

	// The pixelmap containing the heatmap image (Assignment 4)
	QGraphicsPixmapItem *pixmap;
};

#endif
