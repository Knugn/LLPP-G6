
#include "ped_chunks.h"

#include <cassert>
#include <cstddef>
#include <algorithm>

using namespace std;

Ped::Tchunks::Tchunks(int xOffset, int yOffset, int chunkSizeX, int chunkSizeY, int nChunksX, int nChunksY) :
	xOffset(xOffset), yOffset(yOffset), chunkSizeX(chunkSizeX), chunkSizeY(chunkSizeY), nChunksX(nChunksX), nChunksY(nChunksY)
{
	//this->xOffset = xOffset;
	//this->yOffset = yOffset;
	//this->chunkSizeX = chunkSizeX;
	//this->chunkSizeY = chunkSizeY;
	//this->nChunksX = nChunksX;
	//this->nChunksY = nChunksY;
	agents = new std::set<const Ped::Tagent*>[nChunksX*nChunksY];
}

Ped::Tchunks::~Tchunks() {
	delete [] agents;
}

int Ped::Tchunks::getChunkIndex(int chunkIdxX, int chunkIdxY) const {
	chunkIdxX %= nChunksX;
	if (chunkIdxX < 0)
		chunkIdxX += nChunksX;
	chunkIdxY %= nChunksY;
	if (chunkIdxY < 0)
		chunkIdxY += nChunksY;
	return chunkIdxX*nChunksY+chunkIdxY;
}

int Ped::Tchunks::getChunkIndexX(int xPosition) const {
	xPosition -= xOffset;
	int xSize = nChunksX * chunkSizeX;
	xPosition %= xSize;
	if (xPosition < 0)
		xPosition += xSize;
	int xIndex = xPosition / chunkSizeX;
	return xIndex;
}

int Ped::Tchunks::getChunkIndexY(int yPosition) const {
	yPosition -= yOffset;
	int ySize = nChunksY * chunkSizeY;
	yPosition %= ySize;
	if (yPosition < 0)
		yPosition += ySize;
	int yIndex = yPosition / chunkSizeY;
	return yIndex;
}

int Ped::Tchunks::getChunkIndexFromPosition(int x, int y) const {
	return getChunkIndex(getChunkIndexX(x), getChunkIndexY(y));
}


void Ped::Tchunks::clear() {
	int nChunks = nChunksX*nChunksY;
	for (int i=0; i < nChunks; i++) {
		clear(i);
	}
}

void Ped::Tchunks::clear(int chunkIdxX, int chunkIdxY) {
	clear(getChunkIndex(chunkIdxX, chunkIdxY));
}

void Ped::Tchunks::clear(int chunkIndex) {
	set<const Ped::Tagent*> chunkAgents = agents[chunkIndex];
	chunkAgents.clear();
}


void Ped::Tchunks::addAgent(const Ped::Tagent *a) {
	int chunkIndex = getChunkIndexFromPosition(a->getX(), a->getY());
	set<const Ped::Tagent*> chunkAgents = agents[chunkIndex];
	chunkAgents.insert(a);
}

bool Ped::Tchunks::removeAgent(const Ped::Tagent *a) {
	int chunkIndex = getChunkIndexFromPosition(a->getX(), a->getY());
	set<const Ped::Tagent*> chunkAgents = agents[chunkIndex];
	size_t removedCount = chunkAgents.erase(a);
	return (removedCount > 0);
}

bool Ped::Tchunks::moveAgent(const Ped::Tagent *a, int oldX, int oldY) {
	int chunkIdxXOld = getChunkIndexX(oldX);
	int chunkIdxXNew = getChunkIndexX(a->getX());
	int chunkIdxYOld = getChunkIndexX(oldY);
	int chunkIdxYNew = getChunkIndexX(a->getY());
	if (chunkIdxXOld != chunkIdxXNew || chunkIdxYOld != chunkIdxYNew) {
		int oldChunkIdx = getChunkIndex(chunkIdxXOld, chunkIdxYOld);
		int newChunkIdx = getChunkIndex(chunkIdxXNew, chunkIdxYNew);
		agents[oldChunkIdx].erase(a);
		agents[newChunkIdx].insert(a);
		return true;
	}
	return false;
}

set<const Ped::Tagent*> Ped::Tchunks::getAgents(int chunkIdxX, int chunkIdxY) const {
	int chunkIndex = getChunkIndex(chunkIdxX, chunkIdxY);
	set<const Ped::Tagent*> chunkAgents = agents[chunkIndex];
	return chunkAgents;
}


void Ped::Tchunks::getAgents(int chunkIdxX, int chunkIdxY, list<const Ped::Tagent*>& dest) const {
	set<const Ped::Tagent*> chunkAgents = getAgents(chunkIdxX, chunkIdxY);
	for (set<const Ped::Tagent*>::iterator it = chunkAgents.begin(); it != chunkAgents.end(); ++it) {
		const Ped::Tagent* currentAgent = (*it);
		dest.push_back(currentAgent);
	}
}


void Ped::Tchunks::getAgents(int px, int py, int pr, list<const Ped::Tagent*>& dest) const {
	return getAgents(px-pr,px+pr, py-pr, py+pr, dest);
}


void Ped::Tchunks::getAgents(int x1, int x2, int y1, int y2, list<const Ped::Tagent*>& dest) const {
	int xIndexStart = getChunkIndexX(x1);
	int xIndexEnd = getChunkIndexX(x2);
	if (xIndexEnd < xIndexStart)
		xIndexEnd += nChunksX;
	int yIndexStart = getChunkIndexY(y1);
	int yIndexEnd = getChunkIndexY(y2);
	if (yIndexEnd < yIndexStart)
		yIndexEnd += nChunksY;
	for (int xIndex = xIndexStart; xIndex <= xIndexEnd; xIndex++) {
		for (int yIndex = yIndexStart; yIndex <= yIndexEnd; yIndex++) {
			set<const Ped::Tagent*> chunkAgents = getAgents(xIndex, yIndex);
			for (set<const Ped::Tagent*>::iterator it = chunkAgents.begin(); it != chunkAgents.end(); ++it) {
				const Ped::Tagent* currentAgent = (*it);
				int x = currentAgent->getX();
				int y = currentAgent->getY();
				if (x1 <= x && x <= x2 && y1 <= y && y <= y2)
					dest.push_back(currentAgent);
			}
		}
	}
}
