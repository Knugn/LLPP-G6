//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2015
//
#include "ped_agent.h"
#include "ped_waypoint.h"
#include "ped_utils.h"
#include <math.h>
#include "immintrin.h"
#include <iostream>


Ped::Tagent::Tagent(int posX, int posY) {

	Ped::Tagent::init(posX, posY);
}


void Ped::Tagent::init(int posX, int posY) {
	x = new int();
	y = new int();
	*this->x = posX;
	*this->y = posY;
	this->waypoints = new deque<Twaypoint*>();
	//*destination = NULL;
	lastDestination = NULL;
}

//__declspec(align(64)) int destX[8];
//__declspec(align(64)) int destY[8];
void Ped::Tagent::computeNextDesiredPosition() {
	/*for (int i = 0; i < 8; i++){
		if (destination == NULL || destination[i] != NULL){
			return;
		}
	}*/
	bool checkDirectiom[8];
	for (int i = 0; i < 8; i++){
		checkDirectiom[i] =  (destination[i] != NULL);
	}
	int mask = 0xffffffff;
	__m256i allMaks = _mm256_setr_epi32(
		checkDirectiom[0] ? mask : 0,
		checkDirectiom[1] ? mask : 0,
		checkDirectiom[2] ? mask : 0,
		checkDirectiom[3] ? mask : 0,
		checkDirectiom[4] ? mask : 0,
		checkDirectiom[5] ? mask : 0,
		checkDirectiom[6] ? mask : 0,
		checkDirectiom[7] ? mask : 0);


	/*int diffX = ((int)destinations[0]->getx()) - *x;
	int diffY = ((int)destinations[0]->gety()) - *y;
	double len = sqrt(diffX * diffX + diffY * diffY);
	*x = round(*x + diffX / len);
	*y = round(*y + diffY / len);*/

	// Load
	__m256i xi = _mm256_maskload_epi32(x, allMaks);
	__m256i yi = _mm256_maskload_epi32(y, allMaks);

	//Load
	
	
	__m256i destXi = _mm256_maskload_epi32(destX, allMaks);
	__m256i destYi = _mm256_maskload_epi32(destY, allMaks);




	//sub
	__m256i diffXi = _mm256_sub_epi32(destXi, xi);
	__m256i diffYi = _mm256_sub_epi32(destYi, yi);
	
	//pytagoras
	//mul
	__m256i xsq = _mm256_mullo_epi32(diffXi, diffXi);
	__m256i ysq = _mm256_mullo_epi32(diffYi, diffYi);
	//Add to sqrt
	__m256i sum = _mm256_add_epi32(xsq, ysq);

	//int to float
	__m256 sqrt = _mm256_cvtepi32_ps(sum);
	//Sqrt
	__m256 len = _mm256_sqrt_ps(sqrt);
	
	//addition

	__m256 diffXf = _mm256_cvtepi32_ps(diffXi);
	//store result
	__m256 divXf = _mm256_div_ps(diffXf, len);
	//to flot shood round
	__m256i divXi = _mm256_cvtps_epi32(divXf);
	sum = _mm256_add_epi32(xi, divXi);
	//division
	_mm256_maskstore_epi32(x, allMaks, sum);

	

	__m256 diffYf = _mm256_cvtepi32_ps(diffYi);
	//store result
	__m256 divYf = _mm256_div_ps(diffYf, len);
	//to flot shood round
	__m256i divYi = _mm256_cvtps_epi32(divYf);
	//addition
	sum = _mm256_add_epi32(yi, divYi);
	//division
	_mm256_maskstore_epi32(y, allMaks, sum);


/*	for (int i = 0; i < 8; i++){
		x[i] = x[1];
		y[i] = y[1];
	}
	*/
	/*	int diffX = ((int)destinations[0]->getx()) - *x;
	int diffY = ((int)destinations[0]->gety()) - *y;
	double len = sqrt(diffX * diffX + diffY * diffY);
	*x = round(*x + diffX / len);
	*y = round(*y + diffY / len);*/
}


void Ped::Tagent::computeNextDesiredPositionNormal() {
	if (destination == NULL || destination[0] == NULL) {
		// no destination, no need to
		// compute where to move to
		return;
	}

	float diffX = destination[0]->getx() - *x;
	float diffY = destination[0]->gety() - *y;
	float len = sqrt(diffX * diffX + diffY * diffY);
	x[0] = Ped::Utils::round(*x + (diffX / len));
	y[0] = Ped::Utils::round(*y + (diffY / len));
}


void Ped::Tagent::addWaypoint(Twaypoint* wp) {
	waypoints[0].push_back(wp);
}

Ped::Twaypoint* Ped::Tagent::getNextDestination() {
	Ped::Twaypoint* nextDestination = NULL;
	__declspec(align(64)) int agentReachedDestination[8];
	//float length[8];
	bool checkDirectiom[8];
	for (int i = 0; i < 8; i++){
		checkDirectiom[i] = destination[i] != NULL;
		agentReachedDestination[i] = -1;
	}
	int mask = 0xffffffff;
	__m256i allMaks = _mm256_setr_epi32(
		checkDirectiom[0] ? mask : 0,
		checkDirectiom[1] ? mask : 0,
		checkDirectiom[2] ? mask : 0,
		checkDirectiom[3] ? mask : 0,
		checkDirectiom[4] ? mask : 0,
		checkDirectiom[5] ? mask : 0,
		checkDirectiom[6] ? mask : 0,
		checkDirectiom[7] ? mask : 0);
	


		// compute if agent reached its current destination
		__m256i xi = _mm256_maskload_epi32(x, allMaks);
		__m256i yi = _mm256_maskload_epi32(y, allMaks);
		//Load


		__m256i destXi = _mm256_maskload_epi32(destX, allMaks);
		__m256i destYi = _mm256_maskload_epi32(destY, allMaks);


		//sub
		__m256i diffXi = _mm256_sub_epi32(destXi, xi);
		__m256i diffYi = _mm256_sub_epi32(destYi, yi);

		//pytagoras
		//mul
		__m256i xsq = _mm256_mullo_epi32(diffXi, diffXi);
		__m256i ysq = _mm256_mullo_epi32(diffYi, diffYi);
		//Add to sqrt
		__m256i sum = _mm256_add_epi32(xsq, ysq);
		__m256 sqrt = _mm256_cvtepi32_ps(sum);
		//Sqrt
		__m256 len = _mm256_sqrt_ps(sqrt);
		//__m256i leni = _mm256_cvtps_epi32(len);
		__m256 r = _mm256_maskload_ps(destR, allMaks);
		//1: OP : = _CMP_LT_OS
		//29: OP := _CMP_GE_OQ
		//30: OP : = _CMP_GT_OQ
		//think shood be grater than, dont know what OQ mean
		//http://stackoverflow.com/questions/16988199/how-to-choose-avx-compare-predicate-variants
		__m256 ans = _mm256_cmp_ps(len, r, 29);
		__m256i ansi = _mm256_cvtps_epi32(ans);
		_mm256_maskstore_epi32(agentReachedDestination, allMaks, ansi);

		/*for (int i = 0; i < 8; i++){
			if (destination[i] != NULL) {
				double diffX = destination[i]->getx() - x[i];
				double diffY = destination[i]->gety() - y[i];
				double length = sqrt(diffX * diffX + diffY * diffY);
				agentReachedDestination[i] = length < destination[i]->getr();
			}
		}*/
		
	
	for (int i = 0; i < 8; i++){
		if ((agentReachedDestination[i] == 0 || destination[i] == NULL) && !waypoints[i].empty()) {
		// Case 1: agent has reached destination (or has no current destination);
		// get next destination if available
			waypoints[i].push_back(destination[i]);
			nextDestination = waypoints[i].front();
			waypoints[i].pop_front();
		}
		else {
			// Case 2: agent has not yet reached destination, continue to move towards
			// current destination
			nextDestination = destination[i];
		}
		destination[i] = nextDestination;
		if (destination[i] != NULL){
			destX[i] = destination[i]->getx();
			destY[i] = destination[i]->gety();
			destR[i] = destination[i]->getr();
		}
	}
	return nextDestination;
}

Ped::Twaypoint* Ped::Tagent::getNextDestinationNormal() {
	Ped::Twaypoint* nextDestination = NULL;
	bool agentReachedDestination = false;


	if (destination[0] != NULL) {
		// compute if agent reached its current destination
		double diffX = destination[0]->getx() - *x;
		double diffY = destination[0]->gety() - *y;
		double length = sqrt(diffX * diffX + diffY * diffY);
		agentReachedDestination = length < destination[0]->getr();
	}

	if ((agentReachedDestination || destination[0] == NULL) && !waypoints[0].empty()) {
		// Case 1: agent has reached destination (or has no current destination);
		// get next destination if available
		waypoints[0].push_back(destination[0]);
		nextDestination = waypoints[0].front();
		waypoints[0].pop_front();
	}
	else {
		// Case 2: agent has not yet reached destination, continue to move towards
		// current destination
		nextDestination = destination[0];
	}
	destination[0] = nextDestination;
	if (destination[0] != NULL){
		destX[0] = destination[0]->getx();
		destY[0] = destination[0]->gety();
		destR[0] = destination[0]->getr();
	}
	return nextDestination;
}

void Ped::Tagent::computeNextDesiredPositionOrignal() {
	destination[0] = getNextDestinationNormal();
	if (destination == NULL || destination[0] == NULL) {
		// no destination, no need to
		// compute where to move to
		return;
	}

	float diffX = destination[0]->getx() - *x;
	float diffY = destination[0]->gety() - *y;
	float len = sqrt(diffX * diffX + diffY * diffY);
	x[0] = Ped::Utils::round(*x + (diffX / len));
	y[0] = Ped::Utils::round(*y + (diffY / len));
}
