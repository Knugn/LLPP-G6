//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2016
//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2015
//
// TAgent represents an agent in the scenario. Each
// agent has a position (x,y) and a number of destinations
// it wants to visit (waypoints). The desired next position
// represents the position it would like to visit next as it
// will bring it closer to its destination.
// Note: the agent will not move by itself, but the movement
// is handled in ped_model.cpp. 
#ifndef _ped_agent_h_
#define _ped_agent_h_ 1


// g++ does not understand cuda stuff. This makes it ignore them. (use this if you want)
#ifndef __CUDACC__
#define __device__ 
#define __host__
#endif

#include <vector>
#include <deque>

using namespace std;

namespace Ped {
	class Twaypoint;

	class Tagent {
	public:
		Tagent(int posX, int posY);

		// Returns the coordinates of the desired position
		int getDesiredX() const { return *desX; }
		int getDesiredY() const { return *desY; }

		// Sets the agent's position
		void setX(int newX) { *x = newX; }
		void setY(int newY) { *y = newY; }


		void Ped::Tagent::computeNextDesiredPositionOrignal();

		// Update the position according to get closer
		// to the current destination
		void computeNextDesiredPosition();

		// Position of agent defined by x and y
		int getX() const { return *x; };
		int getY() const { return *y; };

		// Adds a new waypoint to reach for this agent
		void addWaypoint(Twaypoint* wp);


		// Returnst he next destination to visit
		Twaypoint* getNextDestination();
		int * x;
		int * y;
		int * desX;
		int * desY;
		int * destX;
		int * destY;
		float * destR;
		// The queue of all destinations that this agent still has to visit
		deque<Twaypoint*> * waypoints;

		//Ugly code, plz dont judge 
		void initValues(int * x, int *y, Twaypoint ** destination, int * destX, int * destY, float * destR, deque<Twaypoint*> * waypoints, int * desx, int * desy){
			*x = *this->x;
			*y = *this->y;

			int * tempX = this->x;
			int * tempY = this->y;

			this->x = x;
			this->y = y;

			this->destination = destination;
			this->destination[0] = NULL;

			this->destX = destX;
			this->destY = destY;
			this->destR = destR;

			this->desX = desx;
			this->desY = desy;

			*waypoints = *this->waypoints;
			deque<Twaypoint*> * tempwaypoints = this->waypoints;
			this->waypoints = waypoints;

			free(tempX);
			free(tempY);
			//free(tempwaypoints);

			//return std::make_pair(*this->x, *this->y);

		}

		void Ped::Tagent::computeNextDesiredPositionNormal();
		Ped::Twaypoint* Ped::Tagent::getNextDestinationNormal();
	private:
		Tagent() {};

		// The agent's current position


		// The agent's desired next position
		//int  desiredPositionX;
		//int  desiredPositionY;

		// The current destination (may require several steps to reach)
		Twaypoint** destination;

		// The last destination
		Twaypoint* lastDestination;



		// Internal init function 
		void init(int posX, int posY);



	};
}

#endif
