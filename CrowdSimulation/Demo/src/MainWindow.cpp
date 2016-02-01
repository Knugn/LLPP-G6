#include <Windows.h>
#include "MainWindow.h"

#include <QGraphicsView>
#include <QtGui>
#include <QBrush>

#include <iostream>

MainWindow::MainWindow(const Ped::Model &model) : model(model)
{

	// The Window 
	graphicsView = new QGraphicsView();

	setCentralWidget(graphicsView);

	// A surface for managing a large number of 2D graphical items
	scene = new QGraphicsScene(QRect(0,0,800,600),this);

	// Connect
	graphicsView->setScene(scene);


	// Paint on surface
	scene->setBackgroundBrush(Qt::black);
	//graphicsscene->setItemIndexMethod(QGraphicsScene::NoIndex);


	for (int x=0; x<=800; x+=cellsizePixel)
	{
		scene->addLine(x,0,x,800, QPen(Qt::gray));
	}

	// Now add the horizontal lines, paint them green
	for (int y=0; y<=800; y+=cellsizePixel)
	{
		scene->addLine(0,y,800,y, QPen(Qt::gray));
	}

	// Create viewAgents with references to the position of the model counterparts
	const std::vector<Ped::Tagent*> &agents = model.getAgents();

	std::vector<Ped::Tagent*>::const_iterator it;
	for (it = agents.begin(); it != agents.end(); it++) 
	{
		viewAgents.push_back(new ViewAgent(*it,scene));  
	}

	////////////
	/// NEW
	///////////////////////////////////////////////
	const int heatmapSize = model.getHeatmapSize();
	QPixmap pixmapDummy = QPixmap(heatmapSize, heatmapSize);
	pixmap = scene->addPixmap(pixmapDummy);
	////////////
	/// END NEW
	///////////////////////////////////////////////

	paint();
	graphicsView->show(); // Redundant? 
}


void MainWindow::paint() {
	////////////
	/// NEW
	///////////////////////////////////////////////
	const int heatmapSize = model.getHeatmapSize();
	QImage image((uchar*) *model.getHeatmap(), heatmapSize, heatmapSize, heatmapSize * sizeof(int), QImage::Format_ARGB32);
	pixmap->setPixmap(QPixmap::fromImage(image));
	////////////
	/// END NEW
	///////////////////////////////////////////////
	
	std::vector<ViewAgent*>::iterator it;
	for (it = viewAgents.begin(); it != viewAgents.end(); it++) 
	{
		(*it)->paint(); 
	}
}


int MainWindow::cellToPixel(int val)
{
	//cout << "conv" << val << " " << val*cellsizePixel << endl;
	return val*cellsizePixel;
}
