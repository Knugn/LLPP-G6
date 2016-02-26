#ifndef _ped_chunks_h_
#define _ped_chunks_h_

#include <set>
#include <list>
#include "ped_agent.h"

using namespace std;

namespace Ped {
	class Tagent;

	class Tchunks {
	public:
		const int chunkSizeX, chunkSizeY;
		const int xOffset, yOffset;
		const int nChunksX, nChunksY;
	private:
		std::set<const Ped::Tagent*>* agents;

	public:
		Tchunks(int xOffset, int yOffset, int chunkSizeX, int chunkSizeY, int nChunksX, int nChunksY);

		virtual ~Tchunks();

		virtual int getChunkIndexX(int xPosition) const;
		virtual int getChunkIndexY(int yPosition) const;
		
		// Clears all chunks
		virtual void clear();

		// Clears the chunk at the specified index
		virtual void clear(int chunkIdxX, int chunkIdxY);

		// Adds an agent a to the chunk containing the agents coordinates.
		virtual void addAgent(const Ped::Tagent *a);
		
		// Removes agent a from the chunk containing the agents coordinates.
		virtual bool removeAgent(const Ped::Tagent *a);

		// Moves an agent to its a new chunk if it crossed a chunk border.
		virtual bool moveAgent(const Ped::Tagent *a, int oldX, int oldY);

		// Returns the set of agents that is stored within this tree
		virtual set<const Ped::Tagent*> getAgents(int chunkIdxX, int chunkIdxY) const;

		// Inserts all agents from the specified chunk into the dest list
		virtual void getAgents(int chunkIdxX, int chunkIdxY, list<const Ped::Tagent*>& dest) const;

		// Inserts all agents inside the bounding box of the given rectangle into the dest list
		virtual void getAgents(int px, int py, int pr, list<const Ped::Tagent*>& dest) const;

		// Inserts all agents inside the given rectangle into the dest list
		virtual void getAgents(int x1, int x2, int y1, int y2, list<const Ped::Tagent*>& dest) const;

	private:
		virtual int getChunkIndex(int chunkIdxX, int chunkIdxY) const;
		virtual int getChunkIndexFromPosition(int x, int y) const;
		
		virtual void clear(int chunkIndex);
	};

}

#endif