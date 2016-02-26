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
#include <iterator>
#include <algorithm>    // std::sort

#define NUM_THREADS 4
std::thread threads[NUM_THREADS];

bool isClSetup = false;
cl::Device clDevice;
cl::Context clContext;
cl::Program clProgram;
cl::Kernel clKernel;
cl::CommandQueue clQueue;
cl::Buffer xPosBuffer, yPosBuffer, xDestBuffer, yDestBuffer, rDestBuffer;


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


void setupOpenClProgram() {
	clDevice = Ped::OpenClUtils::getDefaultDevice();
	Ped::OpenClUtils::printDeviceInfo(clDevice);
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

	std::cout << "Successfully built cl program." << std::endl;
}

void Ped::Model::setup(const std::vector<Ped::Tagent*> &agentsInScenario, IMPLEMENTATION implementation)
{
	agents = std::vector<Ped::Tagent*>(agentsInScenario.begin(), agentsInScenario.end());
	int size = agents.size();
	nRegions = size / 50 + 1;
	std::cout << "Number of agents: " << size << std::endl;
	std::cout << "Number of regions: " << nRegions << std::endl;
	

	xPosistions = (int *)_aligned_malloc(size * sizeof(int), 64);
	int * yPosistions = (int *)_aligned_malloc(size * sizeof(int), 64);

	int * destX = (int *)_aligned_malloc(size * sizeof(int), 64);
	int * destY = (int *)_aligned_malloc(size * sizeof(int), 64);

	int * desiredX = (int *)_aligned_malloc(size * sizeof(int), 64);
	int * desiredY = (int *)_aligned_malloc(size * sizeof(int), 64);

	float * destR = (float *)_aligned_malloc(size * sizeof(float), 64);
	destination = (Twaypoint**)_aligned_malloc(size * sizeof(Twaypoint*), 64);

	__declspec(align(64)) deque<Twaypoint*> * waypoints = new deque<Twaypoint*>[size];


	for (int i = 0; i < size; i++){
		agents[i]->initValues(&(xPosistions[i]), &(yPosistions[i]), &(destination[i]), &(destX[i]), &(destY[i]), &(destR[i]), &(waypoints[i]), &(desiredX[i]), &(desiredY[i]));
	}
	
	
	treehash = new std::map<const Ped::Tagent*, Ped::Ttree*>();

	// Create a new quadtree containing all agents
	tree = new Ttree(NULL, treehash, 0, treeDepth, 0, 0, 1000, 800);
	chunks = new Tchunks(0,0,3,3,1000/3,800/3);
	for (std::vector<Ped::Tagent*>::iterator it = agents.begin(); it != agents.end(); ++it) {
		tree->addAgent(*it);
		chunks->addAgent(*it);
	}

	//implementation = SEQ;
	//implementation = PTHREAD;
	//implementation = OMP;
	//implementation = VECTOR;
	//implementation = OCL;
	//implementation = SEQ_COL;
	//implementation = OMP_COL;
	this->implementation = implementation;


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
		agent->getNextDestinationNormal();
		agent->computeNextDesiredPositionNormal();
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
	err = clQueue.enqueueReadBuffer(yPosBuffer, /*blocking=*/CL_TRUE, 0, sizeof(int)*nAgents, agents[0]->y); 
	Ped::OpenClUtils::checkErr(err, readBuffErrMsg);
}


void Ped::Model::tick_seq_col() {

	int size = agents.size();
	for (int i = 0; i < size; i++) {
		agents[i]->computeNextDesiredPositionOrignal();
	}

	for (int i = 0; i < size; i++) {
		move(agents[i]);
	}
}


void Ped::Model::tick_openmp_col() {

	int size = agents.size();

	for (int i = 0; i < size; i++) {
		agents[i]->computeNextDesiredPositionOrignal();
	}
	std::vector<int> vec(xPosistions, xPosistions + size);
	std::vector<std::vector<Tagent *>> regions(nRegions);
	std::sort(vec.begin(), vec.end());
	std::vector<int> regionsPos(nRegions + 1);
	const int minRegionWidth = chunks->chunkSizeX+1;
	for (int i = 0; i < nRegions; i++){
		regionsPos[i] = vec[(size*i) / nRegions];
		if (i != 0 && regionsPos[i] - regionsPos[i - 1] < minRegionWidth){
			regionsPos[i] = regionsPos[i - 1] + minRegionWidth;
		}
	}
	//regionsPos[nRegions] = vec[size - 1] + 1;
	regionsPos[nRegions] = std::numeric_limits<int>::max();
	
	for (int i = 0; i < size; i++){
		for (int region = 0; region < nRegions; region++){
			if (regionsPos[region] <= agents[i]->getX() && regionsPos[region + 1] > agents[i]->getX()){
				regions[region].push_back(agents[i]);
				break;
			}
		}
	}

	#pragma omp parallel 
	{
		for (int parity=0; parity < 2; parity++) {
			#pragma omp for schedule(dynamic,1)
			for (int i = parity; i < nRegions; i += 2){
				moveRegion(regions[i]);
			}
		}
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
	case Ped::OCL:
		tick_opencl();
		break;
	case Ped::SEQ_COL:
		tick_seq_col();
		break;
	case Ped::OMP_COL:
		tick_openmp_col();
		break;
	default:
		throw new std::runtime_error("Not implemented: " + implementation);
		break;
	}
}
void Ped::Model::moveRegion(std::vector<Ped::Tagent *> regionAgents){
	for (int i = 0; i < regionAgents.size(); i++){
		move(regionAgents[i]);
	}
}

void Ped::Model::move(Ped::Tagent *agent)
{
	// Search for neighboring agents
	//set<const Ped::Tagent *> neighbors = getNeighbors(agent->x[0], agent->y[0], 1);
	set<const Ped::Tagent *> neighbors =getNeighborsFromChunks(agent->getX(), agent->getY(), 1);

	// Retrieve their positions
	std::vector<std::pair<int, int> > takenPositions;
	for (std::set<const Ped::Tagent*>::iterator neighborIt = neighbors.begin(); neighborIt != neighbors.end(); ++neighborIt) {
		std::pair<int, int> position((*neighborIt)->getX(), (*neighborIt)->getY());
		takenPositions.push_back(position);
	}

	// Compute the three alternative positions that would bring the agent
	// closer to his desiredPosition, starting with the desiredPosition itself
	std::vector<std::pair<int, int> > prioritizedAlternatives;
	std::pair<int, int> pDesired(agent->desX[0], agent->desY[0]);
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

			int oldX = agent->getX();
			int oldY = agent->getY();
			// Set the agent's position 
			agent->setX((*it).first);
			agent->setY((*it).second);

			// Update the quadtree
			//(*treehash)[agent]->moveAgent(agent);
			chunks->moveAgent(agent, oldX, oldY);
			break;
		}
	}
}

set<const Ped::Tagent*> Ped::Model::getNeighborsFromChunks(int x, int y, int r) const {
	list<const Ped::Tagent*> neighborList;
	chunks->getAgents(x,y,r,neighborList);
	return set<const Ped::Tagent*>(neighborList.begin(), neighborList.end());
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
