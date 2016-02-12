
__kernel void agent_move(__global const int * xPos, 
						 __global const int * yPos,
						 __global const int * xDest,
						 __global const int * yDest,
						 __global const float * rDest,
						 __global const int nAgents) 
{
	int gid = get_global_id(0);
	if (gid >= nAgents)
		return;
	int dx = xDest[gid] - xPos[gid];
	int dy = yDest[gid] - yPos[gid];
	float dist = sqrt(dx*dx+dy*dy);
	if (dist <= rDest[gid])
		return;
	dx /= dist;
	dy /= dist;
	xPos[gid] += dx;
	yPos[gid] += dy;
}
