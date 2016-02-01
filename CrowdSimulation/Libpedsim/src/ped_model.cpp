//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2015
//
#include "ped_model.h"
#include "ped_agent.h"
#include "ped_waypoint.h"
#include "cuda_dummy.h"
#include "ped_model.h"
#include <iostream>
#include <stack>
#include <algorithm>
#include <stdexcept>
#include <thread>
#include <omp.h>

void Ped::Model::setup(std::vector<Ped::Tagent*> agentsInScenario)
{
	agents = std::vector<Ped::Tagent*>(agentsInScenario.begin(), agentsInScenario.end());
	treehash = new std::map<const Ped::Tagent*, Ped::Ttree*>();

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

	// Set up heatmap (relevant for Assignment 4)
	setupHeatmapSeq();
}

void Ped::Model::tick_seq()
{
	std::vector<Tagent*> agents = getAgents();
	for(std::vector<Tagent*>::iterator iter = agents.begin(); iter != agents.end(); ++iter) {
		Tagent* agent = (*iter);
		agent->computeNextDesiredPosition();
		agent->setX(agent->getDesiredX());
		agent->setY(agent->getDesiredY());
	}
}

void tick_threads_worker(int threadIdx, std::vector<Ped::Tagent*> agents) {
	for (std::size_t idx=threadIdx; idx < agents.size(); idx += NUM_THREADS) {
		Ped::Tagent* agent = agents[idx];
		agent->computeNextDesiredPosition();
		agent->setX(agent->getDesiredX());
		agent->setY(agent->getDesiredY());
	}
}

void Ped::Model::tick_threads() {
	std::vector<Tagent*> agents = getAgents();
	for (int threadIdx = 0; threadIdx < NUM_THREADS; threadIdx++) {
		threads[threadIdx] = std::thread(tick_threads_worker, threadIdx, agents);
	}
	for (int threadIdx = 0; threadIdx < NUM_THREADS; threadIdx++) {
		threads[threadIdx].join();
	}
}

void Ped::Model::tick_openmp() {
	int idx;
	Ped::Tagent* agent;
	std::vector<Tagent*> agents = getAgents();
#pragma omp parallel for shared(agents) private(idx,agent)
	for (idx=0; idx < agents.size(); idx++) {
		agent = agents[idx];
		agent->computeNextDesiredPosition();
		agent->setX(agent->getDesiredX());
		agent->setY(agent->getDesiredY());
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
	case Ped::CUDA:
	case Ped::VECTOR:
	default:
		throw new std::runtime_error("Not implemented: " + implementation);
		break;
	}
}

////////////
/// Everything below here relevant for Assignment 3.
/// Don't use this for Assignment 1!
///////////////////////////////////////////////
void Ped::Model::move( Ped::Tagent *agent)
{
	// Search for neighboring agents
	set<const Ped::Tagent *> neighbors = getNeighbors(agent->getX(), agent->getY(), 2);

	// Retrieve their positions
	std::vector<std::pair<int, int> > takenPositions;
	for (std::set<const Ped::Tagent*>::iterator neighborIt = neighbors.begin(); neighborIt != neighbors.end(); ++neighborIt) {
		std::pair<int,int> position((*neighborIt)->getX(), (*neighborIt)->getY());
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
	if(tree == NULL) 
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
	if(tree != NULL)
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
	while(!treestack.empty()) {
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
	if(tree != NULL)
	{
		delete tree;
		tree = NULL;
	}
	if(treehash != NULL)
	{
		delete treehash;
		treehash = NULL;
	}
}
