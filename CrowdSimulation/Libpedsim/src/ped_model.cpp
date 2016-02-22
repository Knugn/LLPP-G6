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

bool isClSetup = false;
cl::Device clDevice;
cl::Context clContext;
cl::Program clProgram;
cl::Kernel clKernel;
cl::CommandQueue clQueue;
cl::Buffer xPosBuffer, yPosBuffer, xDestBuffer, yDestBuffer, rDestBuffer;

int maxX = 0;
int maxY = 0;
int minX = 0;
int minY = 0;
std::pair <int, int> pairReturned;

std::vector<int> regionX;
std::vector<int> regionY;

vector<int> combinedVec;

std::vector<int> areaBelong;
std::vector<Ped::Tagent*> agentBelong;
std::vector<int> agentsOnBorder;
std::vector<int> agentsInside;
std::vector<int> lastKnownRegion; //if it is on the border it will belong to the old region until it goes to the new one

void setupOpenClProgram() {
	clDevice = Ped::OpenClUtils::getDefaultDevice();
	clContext = Ped::OpenClUtils::createDefaultDeviceContext();
	std::string kernelPath = "..\\Libpedsim\\res\\tick_scalar.cl";
	std::ifstream kernelFile(kernelPath);
	if (!kernelFile.is_open())
		std::cerr << "Failed to open kernel source file \"" << kernelPath << "\"" << std::endl;
	std::string kernelSource((std::istreambuf_iterator<char>(kernelFile)), std::istreambuf_iterator<char>());
	cl::Program::Sources sources;
	sources.push_back(std::make_pair(kernelSource.c_str(), kernelSource.length()));
	clProgram = cl::Program(clContext, sources);
	std::vector<cl::Device> devices;
	devices.push_back(clDevice);
	cl_int err = clProgram.build(devices);
	Ped::OpenClUtils::checkErr(err, ("Failed to build program. Build log: \n" + clProgram.getBuildInfo<CL_PROGRAM_BUILD_LOG>(clDevice)).c_str());
	//if (clProgram.build(devices) != CL_SUCCESS) {
	//	std::cout << " Error building: " << clProgram.getBuildInfo<CL_PROGRAM_BUILD_LOG>(clDevice) << std::endl;
	//	exit(1);
	//}
	std::cout << "Successfully built cl program." << std::endl;

}

void Ped::Model::setup(const std::vector<Ped::Tagent*> &agentsInScenario)
{
	agents = std::vector<Ped::Tagent*>(agentsInScenario.begin(), agentsInScenario.end());
	int size = agents.size();
	int * xPosistions = (int *)_aligned_malloc(size * sizeof(int), 64);
	int * yPosistions = (int *)_aligned_malloc(size * sizeof(int), 64);

	int * destX = (int *)_aligned_malloc(size * sizeof(int), 64);
	int * destY = (int *)_aligned_malloc(size * sizeof(int), 64);

	int * desiredX = (int *)_aligned_malloc(size * sizeof(int), 64);
	int * desiredY = (int *)_aligned_malloc(size * sizeof(int), 64);

	float * destR = (float *)_aligned_malloc(size * sizeof(float), 64);
	destination = (Twaypoint**)_aligned_malloc(size * sizeof(Twaypoint*), 64);

	__declspec(align(64)) deque<Twaypoint*> * waypoints = new deque<Twaypoint*>[size];

	for (int i = 0; i < size; i++){
		pairReturned = agents[i]->updateValus(&(xPosistions[i]), &(yPosistions[i]), &(destination[i]), &(destX[i]), &(destY[i]), &(destR[i]), &(waypoints[i]), &(desiredX[i]), &(desiredY[i]));

		if (pairReturned.first > maxX)
		{
			maxX = pairReturned.first;
		}
		else if (pairReturned.first < minX)
		{
			minX = pairReturned.first;
		}

		if (pairReturned.second > maxY)
		{
			maxY = pairReturned.second;
		}
		else if (pairReturned.second < minY)
		{
			minY = pairReturned.second;
		}
	}

	int x1 = minX;
	int x2 = minX + (std::abs(minX) + std::abs(maxX))*(0.33);
	int x3 = minX + (std::abs(minX) + std::abs(maxX))*(0.66);
	int x4 = maxX;
	int y1 = minY;
	int y2 = minY + (std::abs(minY) + std::abs(maxY))*(0.33);
	int y3 = minY + (std::abs(minY) + std::abs(maxY))*(0.66);
	int y4 = maxY;
	

	/*** super ugly, has to get shorter ***/
	//9 regions are made
	for (int i = 0; i < 3; i++) {
		regionX.push_back(x1);
		regionX.push_back(x2);
		regionX.push_back(x2);
		regionX.push_back(x1);
	}
		
	regionY.push_back(y1);
	regionY.push_back(y1);
	regionY.push_back(y2);
	regionY.push_back(y2);

	regionY.push_back(y2);
	regionY.push_back(y2);
	regionY.push_back(y3);
	regionY.push_back(y3);

	regionY.push_back(y3);
	regionY.push_back(y3);
	regionY.push_back(y4);
	regionY.push_back(y4);

	for (int i = 0; i < 3; i++) {
		regionX.push_back(x2);
		regionX.push_back(x3);
		regionX.push_back(x3);
		regionX.push_back(x2);
	}

	regionY.push_back(y1);
	regionY.push_back(y1);
	regionY.push_back(y2);
	regionY.push_back(y2);

	regionY.push_back(y2);
	regionY.push_back(y2);
	regionY.push_back(y3);
	regionY.push_back(y3);

	regionY.push_back(y3);
	regionY.push_back(y3);
	regionY.push_back(y4);
	regionY.push_back(y4);

	for (int i = 0; i < 3; i++) {
		regionX.push_back(x3);
		regionX.push_back(x4);
		regionX.push_back(x4);
		regionX.push_back(x3);
	}

	regionY.push_back(y1);
	regionY.push_back(y1);
	regionY.push_back(y2);
	regionY.push_back(y2);

	regionY.push_back(y2);
	regionY.push_back(y2);
	regionY.push_back(y3);
	regionY.push_back(y3);

	regionY.push_back(y3);
	regionY.push_back(y3);
	regionY.push_back(y4);
	regionY.push_back(y4);

	/** end of ugly area **/

	//vector size is 36, 4 coordinates foreach of the 9 areas
	int vectorSize = regionX.size();
	int countin = 0;
	int countout = 0;
	bool borderAgentInsertedForThisLoop;

	for (int ag = 0; ag < size; ag++)
	{
		borderAgentInsertedForThisLoop = false;
		

		for (int iter = 0; iter < vectorSize; iter+=4)
		{

			if (regionX[iter] < agents[ag]->x[0] && regionX[iter + 1] > agents[ag]->x[0] && regionY[iter] < agents[ag]->y[0] && regionY[iter + 2] > agents[ag]->y[0])
			{
				int area = (iter / 4) + 1;
				agentsInside.push_back(ag);
				areaBelong.push_back(area); //the area that the agent belongs
				agentBelong.push_back(agents[ag]); //the position of the agent in this vector corresponds to the position of the area in the previous vector
				//std::cout << "agent " << ag << " on iteration " << iter << " is in area " << area << "\n";
				countin++;
			}
			//we can consider borders an other region entirely so we dont have to find the closest region which needs a lot of code
			else if (borderAgentInsertedForThisLoop == false && ((agents[ag]->x[0] == x1) || (agents[ag]->x[0] == x2) || (agents[ag]->x[0] == x3) || (agents[ag]->x[0] == x4)
				|| (agents[ag]->y[0] == y1) || (agents[ag]->y[0] == y2) || (agents[ag]->y[0] == y3) || (agents[ag]->y[0] == y4)))
			{
				agentsOnBorder.push_back(ag);
				areaBelong.push_back(10);
				borderAgentInsertedForThisLoop = true;
				countout++;
			}
		}
	}

	int ss = areaBelong.size();
	std::cout << " agents on the border are " << countout;
	std::cout << " agents INSIDE are " << countin;

	std::cout << "init" << std::endl;
	treehash = new std::map<const Ped::Tagent*, Ped::Ttree*>();

	//destination = new Twaypoint*[agents.size()];

	// Create a new quadtree containing all agents
	tree = new Ttree(NULL, treehash, 0, treeDepth, 0, 0, 1000, 800);
	
	for (std::vector<Ped::Tagent*>::iterator it = agents.begin(); it != agents.end(); ++it)
	{
	tree->addAgent(*it);
	}

	// This is the sequential implementation
	implementation = SEQ;
	//implementation = PTHREAD;
	//implementation = OMP;
	//implementation = VECTOR;
	//implementation = OCL;

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

	//assignment 3
	int tempSize = size - (size % vectorSize);
	//tempSize = 0;

	for (int r = 0; r < 9; r++) {

	}

	for (int i = 0; i < tempSize; i += vectorSize) {
		move(agents[i]);
	}

	for (int i = tempSize; i < size; i++) {
		move(agents[i]);
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



void Ped::Model::tick_opencl() {
	int nAgents = agents.size();
	for (int i = 0; i < nAgents; i++) {
		agents[i]->getNextDestinationNormal();
	}

	if (!isClSetup) {
		setupOpenClProgram();
		cl_int err;
		std::string kernelName = "agent_move";
		clKernel = cl::Kernel(clProgram, "agent_move", &err);
		Ped::OpenClUtils::checkErr(err, ("Failed to locate kernel entry function \"" + kernelName + "\"").c_str());

		const char* createBufferErrMsg = "Failed to create buffer.";
		xPosBuffer = cl::Buffer(clContext, CL_MEM_READ_WRITE, sizeof(int)*nAgents, &err);
		Ped::OpenClUtils::checkErr(err, createBufferErrMsg);
		yPosBuffer = cl::Buffer(clContext, CL_MEM_READ_WRITE, sizeof(int)*nAgents, &err);
		Ped::OpenClUtils::checkErr(err, createBufferErrMsg);
		xDestBuffer = cl::Buffer(clContext, CL_MEM_READ_ONLY, sizeof(int)*nAgents, &err);
		Ped::OpenClUtils::checkErr(err, createBufferErrMsg);
		yDestBuffer = cl::Buffer(clContext, CL_MEM_READ_ONLY, sizeof(int)*nAgents, &err);
		Ped::OpenClUtils::checkErr(err, createBufferErrMsg);
		rDestBuffer = cl::Buffer(clContext, CL_MEM_READ_ONLY, sizeof(float)*nAgents, &err);
		Ped::OpenClUtils::checkErr(err, createBufferErrMsg);

		clQueue = cl::CommandQueue(clContext, clDevice);

		isClSetup = true;
	}

	cl_int err;

	const char* writeBuffErrMsg = "Failed to write to device buffers.";
	err = clQueue.enqueueWriteBuffer(xPosBuffer, CL_FALSE, 0, sizeof(int)*nAgents, agents[0]->x);
	Ped::OpenClUtils::checkErr(err, writeBuffErrMsg);
	err = clQueue.enqueueWriteBuffer(yPosBuffer, CL_FALSE, 0, sizeof(int)*nAgents, agents[0]->y);
	Ped::OpenClUtils::checkErr(err, writeBuffErrMsg);
	err = clQueue.enqueueWriteBuffer(xDestBuffer, CL_FALSE, 0, sizeof(int)*nAgents, agents[0]->destX);
	Ped::OpenClUtils::checkErr(err, writeBuffErrMsg);
	err = clQueue.enqueueWriteBuffer(yDestBuffer, CL_FALSE, 0, sizeof(int)*nAgents, agents[0]->destY);
	Ped::OpenClUtils::checkErr(err, writeBuffErrMsg);
	err = clQueue.enqueueWriteBuffer(rDestBuffer, CL_FALSE, 0, sizeof(float)*nAgents, agents[0]->destR);
	Ped::OpenClUtils::checkErr(err, writeBuffErrMsg);

	const char* setArgsErrMsg = "Failed to set kernel arguments.";
	err = clKernel.setArg(0, xPosBuffer);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clKernel.setArg(1, yPosBuffer);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clKernel.setArg(2, xDestBuffer);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clKernel.setArg(3, yDestBuffer);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clKernel.setArg(4, rDestBuffer);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clKernel.setArg(5, nAgents);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);

	err = clQueue.enqueueNDRangeKernel(clKernel, cl::NullRange, cl::NDRange(nAgents), cl::NullRange);
	Ped::OpenClUtils::checkErr(err, "Failed to enqueue, invoke or run kernel.");

	const char* readBuffErrMsg = "Failed to read buffers back to host.";
	err = clQueue.enqueueReadBuffer(xPosBuffer, CL_FALSE, 0, sizeof(int)*nAgents, agents[0]->x);
	Ped::OpenClUtils::checkErr(err, readBuffErrMsg);
	err = clQueue.enqueueReadBuffer(yPosBuffer, CL_TRUE, 0, sizeof(int)*nAgents, agents[0]->y); //blocking
	Ped::OpenClUtils::checkErr(err, readBuffErrMsg);

	std::cout << "Tick with OpenCL completed." << std::endl;
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
	case Ped::OCL:
		tick_opencl();
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
void Ped::Model::moveNew(Ped::Tagent *agent)
{
	// Search for neighboring agents
	set<const Ped::Tagent *> neighbors = getNeighbors(agent->x[0], agent->y[0], 2);

	// Retrieve their positions
	std::vector<std::pair<int, int> > takenPositions;
	for (std::set<const Ped::Tagent*>::iterator neighborIt = neighbors.begin(); neighborIt != neighbors.end(); ++neighborIt) {
		std::pair<int, int> position((*neighborIt)->getX(), (*neighborIt)->getY());
		takenPositions.push_back(position);
	}

	// Compute the three alternative positions that would bring the agent
	// closer to his desiredPosition, starting with the desiredPosition itself
	std::vector<std::pair<int, int> > prioritizedAlternatives;
	std::pair<int, int> pDesired(agent->desx[0], agent->desy[0]);
	prioritizedAlternatives.push_back(pDesired);

	int diffX = pDesired.first - agent->x[0];
	int diffY = pDesired.second - agent->y[0];
	std::pair<int, int> p1, p2;
	if (diffX == 0 || diffY == 0)
	{
		// Agent wants to walk straight to North, South, West or East
		p1 = std::make_pair(pDesired.first + diffY, pDesired.second + diffX);
		p2 = std::make_pair(pDesired.first - diffY, pDesired.second - diffX);
	}
	else {
		// Agent wants to walk diagonally
		p1 = std::make_pair(pDesired.first, agent->y[0]);
		p2 = std::make_pair(agent->x[0], pDesired.second);
	}
	prioritizedAlternatives.push_back(p1);
	prioritizedAlternatives.push_back(p2);

	// Find the first empty alternative position
	for (std::vector<pair<int, int> >::iterator it = prioritizedAlternatives.begin(); it != prioritizedAlternatives.end(); ++it) {

		// If the current position is not yet taken by any neighbor
		if (std::find(takenPositions.begin(), takenPositions.end(), *it) == takenPositions.end()) {

			// Set the agent's position 
			//agent->setX((*it).first);
			//agent->setY((*it).second);
			agent->x[0] = (*it).first;
			agent->y[0] = (*it).second;

			// Update the quadtree
			(*treehash)[agent]->moveAgent(agent);
			break;
		}
	}
}


void Ped::Model::move(Ped::Tagent *agent)
{
	// Search for neighboring agents
	set<const Ped::Tagent *> neighbors = getNeighbors(agent->x[0], agent->y[0], 2);

	// Retrieve their positions
	std::vector<std::pair<int, int> > takenPositions;
	for (std::set<const Ped::Tagent*>::iterator neighborIt = neighbors.begin(); neighborIt != neighbors.end(); ++neighborIt) {
		std::pair<int, int> position((*neighborIt)->getX(), (*neighborIt)->getY());
		takenPositions.push_back(position);
	}

	// Compute the three alternative positions that would bring the agent
	// closer to his desiredPosition, starting with the desiredPosition itself
	std::vector<std::pair<int, int> > prioritizedAlternatives;
	std::pair<int, int> pDesired(agent->desx[0], agent->desy[0]);
	prioritizedAlternatives.push_back(pDesired);

	int diffX = pDesired.first - agent->x[0];
	int diffY = pDesired.second - agent->y[0];
	std::pair<int, int> p1, p2;
	if (diffX == 0 || diffY == 0)
	{
		// Agent wants to walk straight to North, South, West or East
		p1 = std::make_pair(pDesired.first + diffY, pDesired.second + diffX);
		p2 = std::make_pair(pDesired.first - diffY, pDesired.second - diffX);
	}
	else {
		// Agent wants to walk diagonally
		p1 = std::make_pair(pDesired.first, agent->y[0]);
		p2 = std::make_pair(agent->x[0], pDesired.second);
	}
	prioritizedAlternatives.push_back(p1);
	prioritizedAlternatives.push_back(p2);

	// Find the first empty alternative position
	for (std::vector<pair<int, int> >::iterator it = prioritizedAlternatives.begin(); it != prioritizedAlternatives.end(); ++it) {

		// If the current position is not yet taken by any neighbor
		if (std::find(takenPositions.begin(), takenPositions.end(), *it) == takenPositions.end()) {

			// Set the agent's position 
			//agent->setX((*it).first);
			//agent->setY((*it).second);
			agent->x[0] = (*it).first;
			agent->y[0] = (*it).second;

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
