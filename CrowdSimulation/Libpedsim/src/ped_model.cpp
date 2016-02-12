//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2015
//
#include "ped_model.h"
#include "ped_waypoint.h"
//#include "tick_cuda.h"
#include <iostream>
#include <stack>
#include <algorithm>
#include <thread>
#include <omp.h>
#include "immintrin.h"

//#include "CL/cl.h"
#include "opencl_utils.h"
#include <string>
#include <fstream>
#include <streambuf>


#define NUM_THREADS 4
std::thread threads[NUM_THREADS];

cl::Device clDevice;
cl::Context clContext;
cl::Program clProgram;

void setupOpenClProgram() {
	clDevice = Ped::OpenClUtils::getDefaultDevice();
	clContext = Ped::OpenClUtils::createDefaultDeviceContext();
	std::string kernelPath = "..\\..\\Libpedsim\\res\\tick_scalar.cl";
	std::ifstream t(kernelPath);
	std::string kernelSource((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
	cl::Program::Sources sources;
	sources.push_back(std::make_pair(kernelSource.c_str(),kernelSource.length()));
	clProgram = cl::Program(clContext,sources);
	std::vector<cl::Device> devices;
	devices.push_back(clDevice);
	if (clProgram.build(devices) != CL_SUCCESS) {
		std::cout << " Error building: " << clProgram.getBuildInfo<CL_PROGRAM_BUILD_LOG>(clDevice) << std::endl;
		exit(1);
	}
	std::cout << "Successfully built cl kernel." << std::endl;
}

void Ped::Model::setup(std::vector<Ped::Tagent*> agentsInScenario)
{
	agents = std::vector<Ped::Tagent*>(agentsInScenario.begin(), agentsInScenario.end());
	int size = agents.size();
	int * xPosistions = (int *)_aligned_malloc(size * sizeof(int), 64);
	int * yPosistions = (int *)_aligned_malloc(size * sizeof(int), 64);

	int * destX = (int *)_aligned_malloc(size * sizeof(int), 64);
	int * destY = (int *)_aligned_malloc(size * sizeof(int), 64);
	float * destR = (float *)_aligned_malloc(size * sizeof(float), 64);
	destination = (Twaypoint**)_aligned_malloc(size * sizeof(Twaypoint*), 64);

	__declspec(align(64)) deque<Twaypoint*> * waypoints = new deque<Twaypoint*>[size];

	for (int i = 0; i < size; i++){
		agents[i]->updateValus(&(xPosistions[i]), &(yPosistions[i]), &(destination[i]), &(destX[i]), &(destY[i]), &(destR[i]), &(waypoints[i]));
	}
	std::cout << "init" << std::endl;
	treehash = new std::map<const Ped::Tagent*, Ped::Ttree*>();

	//destination = new Twaypoint*[agents.size()];

	// Create a new quadtree containing all agents
	tree = new Ttree(NULL, treehash, 0, treeDepth, 0, 0, 1000, 800);
	/*
	for (std::vector<Ped::Tagent*>::iterator it = agents.begin(); it != agents.end(); ++it)
	{
	tree->addAgent(*it);
	}*/

	// This is the sequential implementation
	implementation = SEQ;
	//implementation = PTHREAD;
	//implementation = OMP;
	//implementation = VECTOR;

	setupOpenClProgram();

	// Set up heatmap (relevant for Assignment 4)
	setupHeatmapSeq();
}

int vectorSize = 8;
void Ped::Model::tick_seq()
{
	int size = agents.size();
	for (int i = 0; i < size; i++) {
		agents[i]->getNextDestinationNormal();
	}
	for (int i = 0; i < size; i++) {
		agents[i]->computeNextDesiredPositionNormal();
	}
}

void tick_threads_worker(int threadIdx, std::vector<Ped::Tagent*> agents) {
	for (std::size_t idx = threadIdx; idx < agents.size(); idx += NUM_THREADS) {
		Ped::Tagent* agent = agents[idx];
		//		agent->computeNextDesiredPosition();
		agent->computeNextDesiredPositionOrignal();
	}
}

void Ped::Model::tick_threads() {
	for (int threadIdx = 0; threadIdx < NUM_THREADS; threadIdx++) {
		threads[threadIdx] = std::thread(tick_threads_worker, threadIdx, agents);
	}
	for (int threadIdx = 0; threadIdx < NUM_THREADS; threadIdx++) {
		threads[threadIdx].join();
	}
}

void Ped::Model::tick_openmp() {
	omp_set_num_threads(NUM_THREADS);
	int size = agents.size();
	//Vector for getNextDestination works bad
	int tempSize = 0;
	//int tempSize = size - (size % vectorSize);
#pragma omp parallel for 
	for (int i = 0; i < tempSize; i += vectorSize){
		agents[i]->getNextDestination();
	}
#pragma omp parallel for 
	for (int i = tempSize; i < size; i++){
		agents[i]->getNextDestinationNormal();
	}
	//tempSize = size - (size % vectorSize);
#pragma omp parallel for 
	for (int i = 0; i < tempSize; i += vectorSize){
		agents[i]->computeNextDesiredPosition();
	}
#pragma omp parallel for 
	for (int i = tempSize; i < size; i++){
		agents[i]->computeNextDesiredPositionNormal();
	}
}

void Ped::Model::tick_vector() {

	int size = agents.size();

	//int tempSize = 0;
	int tempSize = size - (size % vectorSize);
	for (int i = 0; i < tempSize; i += vectorSize){
		agents[i]->getNextDestination();
	}
	for (int i = tempSize; i < size; i++){
		agents[i]->getNextDestinationNormal();
	}

	//tempSize = size - (size % vectorSize);
	for (int i = 0; i < tempSize; i += vectorSize){
		agents[i]->computeNextDesiredPosition();
	}
	for (int i = tempSize; i < size; i++){
		agents[i]->computeNextDesiredPositionNormal();
	}
}

void Ped::Model::tick()
{
	// EDIT HERE FOR ASSIGNMENT 1
	switch (implementation)
	{
	case Ped::OMP:
		tick_openmp();
		break;
	case Ped::PTHREAD:
		tick_threads();
		break;
	case Ped::SEQ:
		tick_seq();
		break;
	case Ped::VECTOR:
		tick_vector();
		break;
	case Ped::CUDA:
	default:
		throw new std::runtime_error("Not implemented: " + implementation);
		break;
	}
}

////////////
/// Everything below here relevant for Assignment 3.
/// Don't use this for Assignment 1!
///////////////////////////////////////////////
void Ped::Model::move(Ped::Tagent *agent)
{
	// Search for neighboring agents
	set<const Ped::Tagent *> neighbors = getNeighbors(agent->getX(), agent->getY(), 2);

	// Retrieve their positions
	std::vector<std::pair<int, int> > takenPositions;
	for (std::set<const Ped::Tagent*>::iterator neighborIt = neighbors.begin(); neighborIt != neighbors.end(); ++neighborIt) {
		std::pair<int, int> position((*neighborIt)->getX(), (*neighborIt)->getY());
		takenPositions.push_back(position);
	}

	// Compute the three alternative positions that would bring the agent
	// closer to his desiredPosition, starting with the desiredPosition itself
	std::vector<std::pair<int, int> > prioritizedAlternatives;
	std::pair<int, int> pDesired(agent->getDesiredX(), agent->getDesiredY());
	prioritizedAlternatives.push_back(pDesired);

	int diffX = pDesired.first - agent->getX();
	int diffY = pDesired.second - agent->getY();
	std::pair<int, int> p1, p2;
	if (diffX == 0 || diffY == 0)
	{
		// Agent wants to walk straight to North, South, West or East
		p1 = std::make_pair(pDesired.first + diffY, pDesired.second + diffX);
		p2 = std::make_pair(pDesired.first - diffY, pDesired.second - diffX);
	}
	else {
		// Agent wants to walk diagonally
		p1 = std::make_pair(pDesired.first, agent->getY());
		p2 = std::make_pair(agent->getX(), pDesired.second);
	}
	prioritizedAlternatives.push_back(p1);
	prioritizedAlternatives.push_back(p2);

	// Find the first empty alternative position
	for (std::vector<pair<int, int> >::iterator it = prioritizedAlternatives.begin(); it != prioritizedAlternatives.end(); ++it) {

		// If the current position is not yet taken by any neighbor
		if (std::find(takenPositions.begin(), takenPositions.end(), *it) == takenPositions.end()) {

			// Set the agent's position 
			agent->setX((*it).first);
			agent->setY((*it).second);

			// Update the quadtree
			(*treehash)[agent]->moveAgent(agent);
			break;
		}
	}
}

/// Returns the list of neighbors within dist of the point x/y. This
/// can be the position of an agent, but it is not limited to this.
/// \date    2012-01-29
/// \return  The list of neighbors
/// \param   x the x coordinate
/// \param   y the y coordinate
/// \param   dist the distance around x/y that will be searched for agents (search field is a square in the current implementation)
set<const Ped::Tagent*> Ped::Model::getNeighbors(int x, int y, int dist) const {
	// if there is no tree, return all agents
	if (tree == NULL)
		return set<const Ped::Tagent*>(agents.begin(), agents.end());

	// create the output list
	list<const Ped::Tagent*> neighborList;
	getNeighbors(neighborList, x, y, dist);

	// copy the neighbors to a set
	return set<const Ped::Tagent*>(neighborList.begin(), neighborList.end());
}

/// Populates the list of neighbors that can be found around x/y./// This triggers a cleanup of the tree structure. Unused leaf nodes are collected in order to
/// save memory. Ideally cleanup() is called every second, or about every 20 timestep.
/// \date    2012-01-28
void Ped::Model::cleanup() {
	if (tree != NULL)
		tree->cut();
}

/// \date    2012-01-29
/// \param   the list to populate
/// \param   x the x coordinate
/// \param   y the y coordinate
/// \param   dist the distance around x/y that will be searched for agents (search field is a square in the current implementation)
void Ped::Model::getNeighbors(list<const Ped::Tagent*>& neighborList, int x, int y, int dist) const {
	stack<Ped::Ttree*> treestack;

	treestack.push(tree);
	while (!treestack.empty()) {
		Ped::Ttree *t = treestack.top();
		treestack.pop();
		if (t->isleaf) {
			t->getAgents(neighborList);
		}
		else {
			if (t->tree1->intersects(x, y, dist)) treestack.push(t->tree1);
			if (t->tree2->intersects(x, y, dist)) treestack.push(t->tree2);
			if (t->tree3->intersects(x, y, dist)) treestack.push(t->tree3);
			if (t->tree4->intersects(x, y, dist)) treestack.push(t->tree4);
		}
	}
}

Ped::Model::~Model()
{
	if (tree != NULL)
	{
		delete tree;
		tree = NULL;
	}
	if (treehash != NULL)
	{
		delete treehash;
		treehash = NULL;
	}
}
